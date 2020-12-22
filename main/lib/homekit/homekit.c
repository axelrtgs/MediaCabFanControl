#include "homekit.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

void on_update(homekit_characteristic_t *ch, homekit_value_t value, void *context);

homekit_characteristic_t current_temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0.0);
homekit_characteristic_t target_temperature = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE, 22.0, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update));
homekit_characteristic_t units = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS, 0);
homekit_characteristic_t current_state = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, OFF);
homekit_characteristic_t target_state = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE, COOL, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update));
homekit_characteristic_t cooling_threshold = HOMEKIT_CHARACTERISTIC_(COOLING_THRESHOLD_TEMPERATURE, 25.0, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update));
homekit_characteristic_t heating_threshold = HOMEKIT_CHARACTERISTIC_(HEATING_THRESHOLD_TEMPERATURE, 15.0, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update));

homekit_server_config_t config;
homekit_service_t* services[10 + 1];
homekit_service_t** s = services;
homekit_accessory_t *accessories[10];

const uint8_t led_gpio = 2;
const bool led_on = false;

// TODO: Persist state across reboots
void update_state() {
  enum target_state_value targ_state = target_state.value.int_value;
  enum target_state_value cur_state = current_state.value.int_value;
  float curr_temp = current_temperature.value.float_value;
  float target_temp = target_temperature.value.float_value;
  float heat_thresh = heating_threshold.value.float_value;
  float cool_thresh = cooling_threshold.value.float_value;

  if ((targ_state == HEAT && curr_temp < target_temp) ||
    (targ_state == AUTO && curr_temp < heat_thresh)) {
    if (cur_state != HEAT) {
      current_state.value = HOMEKIT_UINT8(HEAT);
      homekit_characteristic_notify(&current_state, current_state.value);
      printf("Temp Low Heating Active");

      //heaterOn();
      //coolerOff();
      //fanOff();
      //fanOn(HEATER_FAN_DELAY);
    }
  } else if ((targ_state == COOL && curr_temp > target_temp) ||
    (targ_state == AUTO && curr_temp > cool_thresh)) {
    if (cur_state != COOL) {
      current_state.value = HOMEKIT_UINT8(COOL);
      homekit_characteristic_notify(&current_state, current_state.value);
      printf("Temp High Cooling Active");

      //coolerOn();
      //heaterOff();
      //fanOff();
      //fanOn(COOLER_FAN_DELAY);
    }
  } else {
    if (cur_state != OFF) {
      current_state.value = HOMEKIT_UINT8(OFF);
      homekit_characteristic_notify(&current_state, current_state.value);
      printf("Temps Good Turning Off");

      //coolerOff();
      //heaterOff();
      //fanOff();
    }
  }
  extern_values->cur_temp = curr_temp;
  extern_values->target_temp = target_temp;
  extern_values->mode = cur_state;
}

void on_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
  update_state();
}

void led_write(bool on) {
  gpio_set_level(led_gpio, on ? 1 : 0);
}

void led_init() {
  gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
  led_write(led_on);
}

void thermostat_identify_task(void *_args) {
  for (int i=0; i<3; i++) {
    for (int j=0; j<2; j++) {
      led_write(true);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      led_write(false);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
  led_write(led_on);

  vTaskDelete(NULL);
}

void thermostat_identify(homekit_value_t _value) {
  printf("LED identify\n");
  xTaskCreate(thermostat_identify_task, "Thermostat identify", 512, NULL, 2, NULL);
}

void temperature_sensor_task(void *_args) {
  float temperature_value;
  while (1) {
    int8_t error = ds18b20_read_temp(&temperature_value);
    if (error == 0) {
      printf("Got readings: temperature %g\n", temperature_value);
      current_temperature.value = HOMEKIT_FLOAT(temperature_value);

      homekit_characteristic_notify(&current_temperature, current_temperature.value);

      update_state();
    } else {
      printf("Couldnt read data from sensor\n");
    }
    vTaskDelay(TEMPERATURE_POLL_PERIOD / portTICK_PERIOD_MS);
  }
}

void thermostat_init() {
  xTaskCreate(temperature_sensor_task, "Thermostat", 2048, NULL, 2, NULL);
}

homekit_accessory_t *accessories[] = {
  HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]) {
    HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
      HOMEKIT_CHARACTERISTIC(NAME, "Fan Control"),
      HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Axelrtgs"),
      HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1234321"),
      HOMEKIT_CHARACTERISTIC(MODEL, "TwoZone"),
      HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
      HOMEKIT_CHARACTERISTIC(IDENTIFY, thermostat_identify),
      NULL,
    }),
    HOMEKIT_SERVICE(THERMOSTAT, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
        HOMEKIT_CHARACTERISTIC(NAME, "Fan Control"),
        &current_temperature,
        &target_temperature,
        &current_state,
        &target_state,
        &cooling_threshold,
        &heating_threshold,
        &units,
        NULL,
    }),
    NULL
  }),
  NULL
};

homekit_server_config_t config = {
  .accessories = accessories,
  .password = "111-11-111",
  .setupId = "1QJ8",
};

void homekit_init(fan_kit* fanKit)
{
  extern_values = fanKit;

  extern_values->cur_temp = current_temperature.value.float_value;
  extern_values->target_temp = target_temperature.value.float_value;
  extern_values->mode = current_state.value.int_value;

  homekit_characteristic_notify(&current_temperature, current_temperature.value);
  homekit_characteristic_notify(&target_temperature, target_temperature.value);
  homekit_characteristic_notify(&current_state, current_state.value);

  led_init();

  if (temperature_init()) {
    thermostat_init();
  }

  homekit_server_init(&config);
}
