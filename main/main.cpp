
#include <memory>

#include "config.h"
#include "httpd.h"
#include "log.h"
#include "mqtt.h"
#include "ota.h"
#include "pid_enhanced.h"
#include "pwm_control.h"
#include "temperature.h"
#include "utilities.h"
#include "wifi_prov_mgr.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern "C" {
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include <cJSON.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <mdns.h>
#include <mqtt_client.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <string.h>
}

namespace {
const char *HOMEKIT_PASSWORD = "111-11-111";
const uint32_t I2C_BUS_SPEED = 400000;
const uint8_t MAX31790_ADDRESS = 0x2F;
const uint32_t TEMPERATURE_POLL_PERIOD_MS = 5000;
const uint16_t MIN_FAN_PWM = 125;

const uint8_t NUM_ZONES = 2;

const int TOLERANCE = 10;
const int BANG_ON = false;
const int BANG_OFF = false;
const PID::tuning_t CONSERVATIVE_TUNE{.Kp = 10, .Ki = 1, .Kd = 0};
const PID::tuning_t AGGRESSIVE_TUNE{.Kp = 20, .Ki = 3, .Kd = 0};
const temperature::config_t ZONE_A_CONFIG{.gpio_pin = 4, .instance_id = 0};
const temperature::config_t ZONE_B_CONFIG{.gpio_pin = 5, .instance_id = 1};

const char *MQTT_CONTROL_TOPIC = "/topic/control-messages";
const char *MQTT_DATA_TOPIC = "/topic/data-messages";
const int MQTT_QOS = 0;
const char *CONFIG_BROKER_URL = "mqtt://192.168.0.50";
} // namespace

static const char *TAG = "main_app";

std::shared_ptr<wifi_prov_mgr::wifi_prov_mgr> mwifi(new wifi_prov_mgr::wifi_prov_mgr());

bool connected = false;
double pwm_output[NUM_ZONES] = {0};
float target_temp[NUM_ZONES] = {25};

esp_mqtt_client_handle_t mqtt_client;

typedef struct data_message {
  bool subscribed = false;
  float temperature[NUM_ZONES];
  float target_temperature[NUM_ZONES];
  const char *pid_profile[NUM_ZONES];
  double pid_output[NUM_ZONES];
  int pwm_duty[NUM_PWM];
  int pwm_target[NUM_PWM];
  int tach_rpm[NUM_FANS];
} data_message_t;

data_message_t data_message;

extern "C" {
void wifi_callback(wifi_prov_mgr::WifiConnectionState_t state);

#define MAX_TOPIC_LEN 256

/* Bookkeeping functions */
static void uptime_publish(void) {
  char topic[MAX_TOPIC_LEN];
  char buf[16];

  /* Only publish uptime when connected, we don't want it to be queued */
  if (!MQTT::mqtt_is_connected())
    return;

  /* Uptime (in seconds) */
  sprintf(buf, "%lld", esp_timer_get_time() / 1000 / 1000);
  snprintf(topic, MAX_TOPIC_LEN, "%s/Uptime", wifi_prov_mgr::wifi_prov_mgr::get_hostname());
  MQTT::mqtt_publish(topic, (uint8_t *)buf, strlen(buf), Config::config_mqtt_qos_get(),
                     Config::config_mqtt_retained_get());

  /* Free memory (in bytes) */
  sprintf(buf, "%u", esp_get_free_heap_size());
  snprintf(topic, MAX_TOPIC_LEN, "%s/FreeMemory", wifi_prov_mgr::wifi_prov_mgr::get_hostname());
  MQTT::mqtt_publish(topic, (uint8_t *)buf, strlen(buf), Config::config_mqtt_qos_get(),
                     Config::config_mqtt_retained_get());
}

static void self_publish(void) {
  char topic[MAX_TOPIC_LEN];
  char *payload;

  /* App version */
  payload = (char *)BLE2MQTT_VER;
  snprintf(topic, MAX_TOPIC_LEN, "%s/Version", wifi_prov_mgr::wifi_prov_mgr::get_hostname());
  MQTT::mqtt_publish(topic, (uint8_t *)payload, strlen(payload), Config::config_mqtt_qos_get(),
                     Config::config_mqtt_retained_get());

  /* Config version */
  payload = (char *)Config::config_version_get();
  snprintf(topic, MAX_TOPIC_LEN, "%s/ConfigVersion", wifi_prov_mgr::wifi_prov_mgr::get_hostname());
  MQTT::mqtt_publish(topic, (uint8_t *)payload, strlen(payload), Config::config_mqtt_qos_get(),
                     Config::config_mqtt_retained_get());

  uptime_publish();
}

/* OTA functions */
static void ota_on_completed(OTA::ERR err) {
  ESP_LOGI(TAG, "Update completed: %s", OTA::err_to_str(err));

  /* All done, restart */
  if (err == OTA::ERR::SUCCESS) {
    vTaskSuspendAll();
    esp_restart();
    xTaskResumeAll();
  }
}

static void _ota_on_completed(OTA::ERR err);

static void ota_on_mqtt(const char *topic, const uint8_t *payload, size_t len, void *ctx) {
  char *url = (char *)malloc(len + 1);
  OTA::TYPE type = static_cast<OTA::TYPE>((int)ctx);
  OTA::ERR err;

  memcpy(url, payload, len);
  url[len] = '\0';
  ESP_LOGI(TAG, "Starting %s update from %s", type == OTA::TYPE::FIRMWARE ? "firmware" : "configuration", url);

  if ((err = OTA::download(type, url, _ota_on_completed)) != OTA::ERR::SUCCESS)
    ESP_LOGE(TAG, "Failed updating: %s", OTA::err_to_str(err));
  free(url);
}

static void ota_subscribe(void) {
  char topic[27];

  /* Register for both a specific topic for this device and a general one */
  sprintf(topic, "%s/OTA/Firmware", wifi_prov_mgr::wifi_prov_mgr::get_hostname());
  MQTT::mqtt_subscribe(topic, 0, ota_on_mqtt, (void *)OTA::TYPE::FIRMWARE, NULL);
  MQTT::mqtt_subscribe("BLE2MQTT/OTA/Firmware", 0, ota_on_mqtt, (void *)OTA::TYPE::FIRMWARE, NULL);

  sprintf(topic, "%s/OTA/Config", wifi_prov_mgr::wifi_prov_mgr::get_hostname());
  MQTT::mqtt_subscribe(topic, 0, ota_on_mqtt, (void *)OTA::TYPE::CONFIG, NULL);
  MQTT::mqtt_subscribe("BLE2MQTT/OTA/Config", 0, ota_on_mqtt, (void *)OTA::TYPE::CONFIG, NULL);
}

static void ota_unsubscribe(void) {
  char topic[27];

  sprintf(topic, "%s/OTA/Firmware", wifi_prov_mgr::wifi_prov_mgr::get_hostname());
  MQTT::mqtt_unsubscribe(topic);
  MQTT::mqtt_unsubscribe("BLE2MQTT/OTA/Firmware");

  sprintf(topic, "%s/OTA/Config", wifi_prov_mgr::wifi_prov_mgr::get_hostname());
  MQTT::mqtt_unsubscribe(topic);
  MQTT::mqtt_unsubscribe("BLE2MQTT/OTA/Config");
}

static void cleanup(void) { ota_unsubscribe(); }

/* Network callback functions */
static void network_on_connected(void) {
  Log::log_start(Config::config_log_host_get(), Config::config_log_port_get());
  ESP_LOGI(TAG, "Connected to the network, connecting to MQTT");
  MQTT::mqtt_connect(Config::config_mqtt_host_get(), Config::config_mqtt_port_get(),
                     Config::config_mqtt_client_id_get(), Config::config_mqtt_username_get(),
                     Config::config_mqtt_password_get(), Config::config_mqtt_ssl_get(),
                     Config::config_mqtt_server_cert_get(), Config::config_mqtt_client_cert_get(),
                     Config::config_mqtt_client_key_get());
}

static void network_on_disconnected(void) {
  Log::log_stop();
  ESP_LOGI(TAG, "Disconnected from the network, stopping MQTT");
  MQTT::mqtt_disconnect();
  /* We don't get notified when manually stopping MQTT */
  cleanup();
}

/* MQTT callback functions */
static void mqtt_on_connected(void) {
  ESP_LOGI(TAG, "Connected to MQTT, scanning for BLE devices");
  self_publish();
  ota_subscribe();
}

static void mqtt_on_disconnected(void) {
  static uint8_t num_disconnections = 0;

  ESP_LOGI(TAG, "Disconnected from MQTT, stopping BLE");
  cleanup();

  if (++num_disconnections % 3 == 0) {
    ESP_LOGI(TAG, "Failed connecting to MQTT 3 times, reconnecting to the network");
    // wifi_reconnect();
  }
}

/* BLE2MQTT Task and event callbacks */
typedef enum {
  EVENT_TYPE_HEARTBEAT_TIMER,
  EVENT_TYPE_NETWORK_CONNECTED,
  EVENT_TYPE_NETWORK_DISCONNECTED,
  EVENT_TYPE_OTA_COMPLETED,
  EVENT_TYPE_MQTT_CONNECTED,
  EVENT_TYPE_MQTT_DISCONNECTED,
} event_type_t;

typedef struct {
  event_type_t type;
  union {
    struct {
      OTA::TYPE type;
      OTA::ERR err;
    } ota_completed;
  };
} event_t;

static QueueHandle_t event_queue;

static void ble2mqtt_handle_event(event_t *event) {
  switch (event->type) {
  case EVENT_TYPE_HEARTBEAT_TIMER:
    uptime_publish();
    break;
  case EVENT_TYPE_NETWORK_CONNECTED:
    network_on_connected();
    break;
  case EVENT_TYPE_NETWORK_DISCONNECTED:
    network_on_disconnected();
    break;
  case EVENT_TYPE_OTA_COMPLETED:
    ota_on_completed(event->ota_completed.err);
    break;
  case EVENT_TYPE_MQTT_CONNECTED:
    mqtt_on_connected();
    break;
  case EVENT_TYPE_MQTT_DISCONNECTED:
    mqtt_on_disconnected();
    break;
  }
  free(event);
}

static void ble2mqtt_task(void *pvParameter) {
  event_t *event;

  while (1) {
    if (xQueueReceive(event_queue, &event, portMAX_DELAY) != pdTRUE)
      continue;

    ble2mqtt_handle_event(event);
  }

  vTaskDelete(NULL);
}

static void heartbeat_timer_cb(TimerHandle_t xTimer) {
  event_t *event = (event_t *)malloc(sizeof(*event));

  event->type = EVENT_TYPE_HEARTBEAT_TIMER;

  ESP_LOGD(TAG, "Queuing event HEARTBEAT_TIMER");
  xQueueSend(event_queue, &event, portMAX_DELAY);
}

static int start_ble2mqtt_task(void) {
  TimerHandle_t hb_timer;

  if (!(event_queue = xQueueCreate(10, sizeof(event_t *))))
    return -1;

  if (xTaskCreatePinnedToCore(ble2mqtt_task, "ble2mqtt_task", 4096, NULL, 5, NULL, 1) != pdPASS) {
    return -1;
  }

  hb_timer = xTimerCreate("heartbeat", pdMS_TO_TICKS(60 * 1000), pdTRUE, NULL, heartbeat_timer_cb);
  xTimerStart(hb_timer, 0);

  return 0;
}

static void _network_on_connected(void) {
  event_t *event = (event_t *)malloc(sizeof(*event));

  event->type = EVENT_TYPE_NETWORK_CONNECTED;

  ESP_LOGD(TAG, "Queuing event NETWORK_CONNECTED");
  xQueueSend(event_queue, &event, portMAX_DELAY);
}

static void _network_on_disconnected(void) {
  event_t *event = (event_t *)malloc(sizeof(*event));

  event->type = EVENT_TYPE_NETWORK_DISCONNECTED;

  ESP_LOGD(TAG, "Queuing event NETWORK_DISCONNECTED");
  xQueueSend(event_queue, &event, portMAX_DELAY);
}

static void _ota_on_completed(OTA::ERR err) {
  event_t *event = (event_t *)malloc(sizeof(*event));

  event->type = EVENT_TYPE_OTA_COMPLETED;
  // event->ota_completed.type = type;
  event->ota_completed.err = err;

  ESP_LOGD(TAG, "Queuing event EVENT_TYPE_OTA_COMPLETED (%d)", (int)err);
  xQueueSend(event_queue, &event, portMAX_DELAY);
}

static void _mqtt_on_connected(void) {
  event_t *event = (event_t *)malloc(sizeof(*event));

  event->type = EVENT_TYPE_MQTT_CONNECTED;

  ESP_LOGD(TAG, "Queuing event MQTT_CONNECTED");
  xQueueSend(event_queue, &event, portMAX_DELAY);
}

static void _mqtt_on_disconnected(void) {
  event_t *event = (event_t *)malloc(sizeof(*event));

  event->type = EVENT_TYPE_MQTT_DISCONNECTED;

  ESP_LOGD(TAG, "Queuing event MQTT_DISCONNECTED");
  xQueueSend(event_queue, &event, portMAX_DELAY);
}

void app_main() {
  /* Setup Loggin */
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("main_app", ESP_LOG_DEBUG);
  esp_log_level_set("wifi_prov_mgr", ESP_LOG_DEBUG);
  esp_log_level_set("owb", ESP_LOG_INFO);
  esp_log_level_set("owb_rmt", ESP_LOG_INFO);
  esp_log_level_set("ds18b20", ESP_LOG_INFO);

  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  ESP_LOGD(TAG, "This is ESP32 chip with %d CPU cores, WiFi%s%s, silicon revision %d, %dMB %s flash", chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
           chip_info.revision, spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  esp_err_t ret;

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGD(TAG, "No free pages erasing flash");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGD(TAG, "Setting WiFi Callback");
  mwifi->registerCallback(wifi_callback);

  ESP_LOGD(TAG, "Starting WiFi");
  mwifi->init(HOMEKIT_PASSWORD);
}

void temperature_sensor_task(void *pvParameters) {
  temperature::config_t *zone_data = (temperature::config_t *)pvParameters;
  uint8_t zone_id = zone_data->instance_id;

  ESP_LOGD(TAG, "Starting temperature_sensor_task %d", zone_id);
  std::shared_ptr<temperature::temperature> temperature(new temperature::temperature());

  temperature->init(zone_data);
  PID::PIDEnhanced *PID =
      new PID::PIDEnhanced(TOLERANCE, BANG_ON, BANG_OFF, MIN_FAN_PWM, PWM_MAX, CONSERVATIVE_TUNE, AGGRESSIVE_TUNE);
  while (1) {
    auto average_temperature = temperature->average_temperature();
    ESP_LOGD(TAG, "Temperature zone %d: %f", zone_id, average_temperature);
    ESP_LOGD(TAG, "Target temperature zone %d: %f", zone_id, target_temp[zone_id]);

    double PID_output;
    std::string PID_Profile = PID->compute(average_temperature, target_temp[zone_id], &PID_output);

    pwm_output[zone_id] = PID_output;

    ESP_LOGD(TAG, "PID Profile zone %d: %s", zone_id, PID_Profile.c_str());
    ESP_LOGD(TAG, "PID Out zone %d: %f", zone_id, PID_output);

    data_message.temperature[zone_id] = average_temperature;
    data_message.target_temperature[zone_id] = target_temp[zone_id];
    data_message.pid_profile[zone_id] = PID_Profile.c_str();
    data_message.pid_output[zone_id] = PID_output;

    vTaskDelay(TEMPERATURE_POLL_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void handle_control_msg(esp_mqtt_event_handle_t event) {
  char topic[event->topic_len];
  char data[event->data_len];
  memcpy(topic, event->topic, event->topic_len);
  topic[event->topic_len] = '\0';
  memcpy(data, event->data, event->data_len);
  data[event->data_len] = '\0';

  ESP_LOGD(TAG, "MQTT TOPIC=%s", topic);
  ESP_LOGD(TAG, "MQTT DATA=%s", data);

  if (strcmp(topic, MQTT_CONTROL_TOPIC) == 0) {
    cJSON *json = cJSON_Parse(data);
    if (json == NULL) {
      const char *error_ptr = cJSON_GetErrorPtr();
      if (error_ptr != NULL) {
        ESP_LOGE(TAG, "JSON Control message syntax error.");
      }
    } else {
      const cJSON *target = NULL;
      const cJSON *targets = NULL;
      targets = cJSON_GetObjectItemCaseSensitive(json, "target_temps");
      int zone_id = 0;
      cJSON_ArrayForEach(target, targets) {
        if (cJSON_IsNumber(target)) {
          float temp = target->valuedouble;

          ESP_LOGD(TAG, "JSON Control temperature target zone %d: %f", zone_id, temp);
          target_temp[zone_id] = temp;
          zone_id++;
        }
      }
    }
    cJSON_Delete(json);
  } else {
    ESP_LOGI(TAG, "Unhandled topic: %s", topic);
  }
}

void publish_data_msg(data_message_t *values) {
  cJSON *root = cJSON_CreateObject();
  cJSON_AddBoolToObject(root, "subscribed", values->subscribed);
  cJSON_AddItemToObject(root, "temperature", cJSON_CreateFloatArray(values->temperature, NUM_ZONES));
  cJSON_AddItemToObject(root, "target_temperature", cJSON_CreateFloatArray(values->target_temperature, NUM_ZONES));
  cJSON_AddItemToObject(root, "pid_profile", cJSON_CreateStringArray(values->pid_profile, NUM_ZONES));
  cJSON_AddItemToObject(root, "pid_output", cJSON_CreateDoubleArray(values->pid_output, NUM_ZONES));
  cJSON_AddItemToObject(root, "pwm_duty", cJSON_CreateIntArray(values->pwm_duty, NUM_PWM));
  cJSON_AddItemToObject(root, "pwm_target", cJSON_CreateIntArray(values->pwm_target, NUM_PWM));
  cJSON_AddItemToObject(root, "tach_rpm", cJSON_CreateIntArray(values->tach_rpm, NUM_FANS));

  const char *my_json_string = cJSON_PrintUnformatted(root);

  ESP_LOGD(TAG, "mqtt json string:\n%s", my_json_string);

  esp_mqtt_client_publish(mqtt_client, MQTT_DATA_TOPIC, my_json_string, 0, MQTT_QOS, false);
  cJSON_Delete(root);
}

void pwm_control_task(void *_pvParameters) {
  ESP_LOGD(TAG, "Starting pwm_control_task");

  I2C_t &myI2C = i2c0; // i2c0 and i2c1 are the default objects
  ESP_ERROR_CHECK(myI2C.begin(GPIO_NUM_21, GPIO_NUM_22, GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE, I2C_BUS_SPEED));
  myI2C.setTimeout(10);
  ESP_LOGD(TAG, "I2C initialized");

  ESP_ERROR_CHECK(myI2C.testConnection(MAX31790_ADDRESS));
  ESP_LOGD(TAG, "I2C connection tested");

  PWMControl::PWMControl *pwmControl = new PWMControl::PWMControl(&myI2C, MAX31790_ADDRESS);
  ESP_ERROR_CHECK(pwmControl->init());
  ESP_LOGD(TAG, "PWMControl Initialized");
  while (1) {
    uint16_t pwmDuty[NUM_PWM];
    pwmControl->getPWMDutyComplete(pwmDuty);

    for (auto it = std::begin(pwmDuty); it != std::end(pwmDuty); ++it) {
      ESP_LOGD(TAG, "PWM duty is: %d", *it);
    }

    uint16_t pwm_output_a = static_cast<uint16_t>(pwm_output[0]);
    uint16_t pwm_output_b = static_cast<uint16_t>(pwm_output[1]);
    uint16_t pwmNewTarget[] = {pwm_output_a, pwm_output_a, pwm_output_b, pwm_output_b};
    pwmControl->setPWMTargetComplete(pwmNewTarget);

    uint16_t pwmTarget[NUM_PWM];
    pwmControl->getPWMTargetComplete(pwmTarget);

    for (auto it = std::begin(pwmTarget); it != std::end(pwmTarget); ++it) {
      ESP_LOGD(TAG, "New PWM target is: %d", *it);
    }

    uint16_t tachRPM[NUM_FANS];
    pwmControl->getTachRPMComplete(tachRPM);

    for (auto it = std::begin(tachRPM); it != std::end(tachRPM); ++it) {
      ESP_LOGD(TAG, "Tach RPM is: %d", *it);
    }

    std::copy(std::begin(pwmDuty), std::end(pwmDuty), std::begin(data_message.pwm_duty));
    std::copy(std::begin(pwmTarget), std::end(pwmTarget), std::begin(data_message.pwm_target));
    std::copy(std::begin(tachRPM), std::end(tachRPM), std::begin(data_message.tach_rpm));

    publish_data_msg(&data_message);

    vTaskDelay(TEMPERATURE_POLL_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

static void log_error_if_nonzero(const char *message, int error_code) {
  if (error_code != 0) {
    ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
  }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  // your_context_t *context = event->context;
  switch (event->event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

    msg_id = esp_mqtt_client_subscribe(client, MQTT_CONTROL_TOPIC, MQTT_QOS);
    ESP_LOGD(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    data_message.subscribed = true;
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    data_message.subscribed = false;
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGD(TAG, "MQTT_EVENT_DATA");
    handle_control_msg(event);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_ESP_TLS) {
      log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
      log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
      log_error_if_nonzero("captured as transport's socket errno", event->error_handle->connect_return_code);
      ESP_LOGE(TAG, "Last errno string (%s)", strerror(event->error_handle->connect_return_code));
    }
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
  return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
  mqtt_event_handler_cb((esp_mqtt_event_handle_t)event_data);
}

static void mqtt_app_start(void) {
  esp_mqtt_client_config_t mqtt_cfg = {
      .uri = CONFIG_BROKER_URL,
  };

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, mqtt_client);
  esp_mqtt_client_start(mqtt_client);
}

void wifi_callback(wifi_prov_mgr::WifiConnectionState_t state) {
  ESP_LOGD(TAG, "Wifi callback state: %d", (int)state.wifiState);
  if (state.wifiState == wifi_prov_mgr::WifiAssociationState::CONNECTED && !connected) {
    ESP_LOGI(TAG, "Wifi connected initializing controller");
    connected = true;
    int config_failed;
    ESP_LOGI(TAG, "Version: %s", BLE2MQTT_VER);

    /* Init configuration */
    config_failed = Config::config_initialize();

    /* Init mDNS */
    ESP_ERROR_CHECK(mdns_init());
    mdns_hostname_set(wifi_prov_mgr::wifi_prov_mgr::get_hostname());

    /* Init MQTT */
    // ESP_ERROR_CHECK(mqtt_initialize());
    // mqtt_set_on_connected_cb(_mqtt_on_connected);
    // mqtt_set_on_disconnected_cb(_mqtt_on_disconnected);

    /* Init web server */
    httpd::httpd *http_server = new httpd::httpd();
    ESP_ERROR_CHECK(http_server->init());
    http_server->set_on_ota_completed_cb(_ota_on_completed);

    /* Start BLE2MQTT task */
    // ESP_ERROR_CHECK(start_ble2mqtt_task());

    xTaskCreate(temperature_sensor_task, "ZONE_A", 2560, (void *)&ZONE_A_CONFIG, 2, NULL);
    xTaskCreate(temperature_sensor_task, "ZONE_B", 2560, (void *)&ZONE_B_CONFIG, 2, NULL);
    vTaskDelay((TEMPERATURE_POLL_PERIOD_MS * 2) / portTICK_PERIOD_MS);
    xTaskCreate(pwm_control_task, "PWM_CONTROL", 2560, NULL, 2, NULL);

    mqtt_app_start();
  } else if (state.wifiState == wifi_prov_mgr::WifiAssociationState::DISCONNECTED && connected) {
    ESP_LOGI(TAG, "Wifi disconnected");
    connected = false;
  }
}
}
