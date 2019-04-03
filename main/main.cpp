#define CONFIG_I2CBUS_LOG_READWRITES
#define CONFIG_I2CBUS_LOG_RW_LEVEL_INFO

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "FanControl.h"
#include "PIDEnhanced.h"

namespace
{
const uint32_t I2C_BUS_SPEED = 400000;
const uint8_t MAX31790_ADDRESS = 0x2F;
}

extern "C" void app_main()
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n",
           spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf(">I2Cbus Example \n");
    fflush(stdout);

    I2C_t& myI2C = i2c0;  // i2c0 and i2c1 are the default objects

    ESP_ERROR_CHECK(myI2C.begin(GPIO_NUM_21, GPIO_NUM_22, GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE, I2C_BUS_SPEED));
    myI2C.setTimeout(10);

    ESP_ERROR_CHECK(myI2C.testConnection(MAX31790_ADDRESS));

    FanControl* fanControl = new FanControl(&myI2C, MAX31790_ADDRESS);
    ESP_ERROR_CHECK(fanControl->init());

    PIDEnhanced::PID_TUNING conservative_tune
    {
        .Kp = -10.0,
        .Ki = 0.0,
        .Kd = 0.0
    };

    PIDEnhanced::PID_TUNING aggressive_tune
    {
        .Kp = -10.0,
        .Ki = 0.0,
        .Kd = 0.0
    };
    PIDEnhanced*   _PID;
    std::vector<double> inputVector = {50.0, 50.0};
    double   _PID_target = 50.0;
    double   _PID_output;

    _PID = new PIDEnhanced(10, 0, 511, conservative_tune, aggressive_tune);
    std::string PID_Profile = _PID->computeAvgOfVector(inputVector, _PID_target, &_PID_output);

    printf("PID Profile: %s \n", PID_Profile.c_str());

    printf("PID Out: %f \n", _PID_output);

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
