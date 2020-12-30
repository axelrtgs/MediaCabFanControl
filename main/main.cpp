
#include <memory>

#include "pid_enhanced.h"
#include "pwm_control.h"
#include "wifi_prov_mgr.h"

extern "C" {
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "temperature.h"
}

namespace {
const char* HOMEKIT_PASSWORD = "111-11-111";
const uint32_t I2C_BUS_SPEED = 400000;
const uint8_t MAX31790_ADDRESS = 0x2F;
const uint32_t TEMPERATURE_POLL_PERIOD_MS = 5000;
const float TARGET_TEMP = 25.0;
const uint16_t MIN_FAN_PWM = 125;

const int TOLERANCE = 10;
const int BANG_ON = false;
const int BANG_OFF = false;
const PID::tuning_t CONSERVATIVE_TUNE{.Kp = 10, .Ki = 1, .Kd = 0};
const PID::tuning_t AGGRESSIVE_TUNE{.Kp = 20, .Ki = 3, .Kd = 0};
const temperature::config_t ZONE_A_CONFIG{.gpio_pin = 4, .instance_id = 0};
const temperature::config_t ZONE_B_CONFIG{.gpio_pin = 5, .instance_id = 1};
}  // namespace

static const char* TAG = "main_app";

std::shared_ptr<wifi_prov_mgr::wifi_prov_mgr> mwifi(
    new wifi_prov_mgr::wifi_prov_mgr());

bool connected = false;
double pwm_output[2] = {0};

extern "C" {
void wifi_callback(wifi_prov_mgr::WifiConnectionState_t state);

void app_main() {
  /* Setup Loggin */
  esp_log_level_set("*", ESP_LOG_DEBUG);
  esp_log_level_set("owb", ESP_LOG_INFO);
  esp_log_level_set("owb_rmt", ESP_LOG_INFO);
  esp_log_level_set("ds18b20", ESP_LOG_INFO);

  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  ESP_LOGD(
      TAG,
      "This is ESP32 chip with %d CPU cores, WiFi%s%s, silicon revision %d, "
      "%dMB %s flash",
      chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
      (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "", chip_info.revision,
      spi_flash_get_chip_size() / (1024 * 1024),
      (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  esp_err_t ret;

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGD(TAG, "No free pages erasing flash");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  mwifi->registerCallback(wifi_callback);
  mwifi->init(HOMEKIT_PASSWORD);
}

void temperature_sensor_task(void* pvParameters) {
  temperature::config_t* zone_data = (temperature::config_t*)pvParameters;
  uint8_t zone_id = zone_data->instance_id;

  ESP_LOGD(TAG, "Starting temperature_sensor_task %d", zone_id);
  std::shared_ptr<temperature::temperature> temperature(
      new temperature::temperature());

  temperature->init(zone_data);
  PID::PIDEnhanced* PID =
      new PID::PIDEnhanced(TOLERANCE, BANG_ON, BANG_OFF, MIN_FAN_PWM, PWM_MAX,
                           CONSERVATIVE_TUNE, AGGRESSIVE_TUNE);
  while (1) {
    auto average_temperature = temperature->average_temperature();
    ESP_LOGD(TAG, "Temperature zone %d: %f", zone_id, average_temperature);

    double PID_output;
    std::string PID_Profile =
        PID->compute(average_temperature, TARGET_TEMP, &PID_output);

    pwm_output[zone_id] = PID_output;

    ESP_LOGD(TAG, "PID Profile zone %d: %s", zone_id, PID_Profile.c_str());
    ESP_LOGD(TAG, "PID Out zone %d: %f", zone_id, PID_output);
    vTaskDelay(TEMPERATURE_POLL_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void pwm_control_task(void* _pvParameters) {
  ESP_LOGD(TAG, "Starting pwm_control_task");

  I2C_t& myI2C = i2c0;  // i2c0 and i2c1 are the default objects
  ESP_ERROR_CHECK(myI2C.begin(GPIO_NUM_21, GPIO_NUM_22, GPIO_PULLUP_DISABLE,
                              GPIO_PULLUP_DISABLE, I2C_BUS_SPEED));
  myI2C.setTimeout(10);
  ESP_LOGD(TAG, "I2C initialized");

  ESP_ERROR_CHECK(myI2C.testConnection(MAX31790_ADDRESS));
  ESP_LOGD(TAG, "I2C connection tested");

  PWMControl::PWMControl* pwmControl =
      new PWMControl::PWMControl(&myI2C, MAX31790_ADDRESS);
  ESP_ERROR_CHECK(pwmControl->init());
  ESP_LOGD(TAG, "PWMControl Initialized");
  while (1) {
    uint16_t pwmDuty[NUM_PWM];
    pwmControl->getPWMDutyComplete(pwmDuty);

    for (auto it = std::begin(pwmDuty); it != std::end(pwmDuty); ++it) {
      ESP_LOGD(TAG, "PWM duty is: %d\n", *it);
    }

    uint16_t pwm_output_a = static_cast<uint16_t>(pwm_output[0]);
    uint16_t pwm_output_b = static_cast<uint16_t>(pwm_output[1]);
    uint16_t pwmNewTarget[] = {pwm_output_a, pwm_output_a, pwm_output_b,
                               pwm_output_b};
    pwmControl->setPWMTargetComplete(pwmNewTarget);

    uint16_t pwmTarget[NUM_PWM];
    pwmControl->getPWMTargetComplete(pwmTarget);

    for (auto it = std::begin(pwmTarget); it != std::end(pwmTarget); ++it) {
      ESP_LOGD(TAG, "New PWM target is: %d\n", *it);
    }

    uint16_t tachRPM[NUM_FANS];
    pwmControl->getTachRPMComplete(tachRPM);

    for (auto it = std::begin(tachRPM); it != std::end(tachRPM); ++it) {
      ESP_LOGD(TAG, "Tach RPM is: %d\n", *it);
    }
    vTaskDelay(TEMPERATURE_POLL_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void wifi_callback(wifi_prov_mgr::WifiConnectionState_t state) {
  ESP_LOGD(TAG, "Wifi callback state: %d", (int)state.wifiState);
  if (state.wifiState == wifi_prov_mgr::WifiAssociationState::CONNECTED &&
      !connected) {
    ESP_LOGI(TAG, "Wifi connected initializing controller");
    connected = true;
    xTaskCreate(temperature_sensor_task, "ZONE_A", 2560, (void*)&ZONE_A_CONFIG,
                2, NULL);
    xTaskCreate(temperature_sensor_task, "ZONE_B", 2560,
    (void*)&ZONE_B_CONFIG,
               2, NULL);
    xTaskCreate(pwm_control_task, "PWM_CONTROL", 2560, NULL, 2, NULL);
  } else if (state.wifiState ==
                 wifi_prov_mgr::WifiAssociationState::DISCONNECTED &&
             connected) {
    ESP_LOGI(TAG, "Wifi disconnected");
    connected = false;
  }
}
}
