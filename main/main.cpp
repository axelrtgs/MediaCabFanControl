#define CONFIG_I2CBUS_LOG_READWRITES
#define CONFIG_I2CBUS_LOG_RW_LEVEL_INFO

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "FanControl.h"
#include "PIDEnhanced.h"

#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "blufi_security.h"
#include "blufi_wireless.h"


namespace
{
const uint32_t I2C_BUS_SPEED = 400000;
const uint8_t MAX31790_ADDRESS = 0x2F;
static const char *TAG = "MAIN";
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

    fflush(stdout);

    esp_err_t ret;
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ret = blufi_initialise_wifi();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Wifi configured disabling bluetooth onboarding.");
        return;
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize bt controller failed: %s\n", __func__, esp_err_to_name(ret));
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "BD ADDR: " ESP_BD_ADDR_STR "\n", ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));

    ESP_LOGI(TAG, "BLUFI VERSION %04x\n", esp_blufi_get_version());

    ret = blufi_security_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init blufi security failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    blufi_init();

    printf("Starting FanControl \n");
    I2C_t& myI2C = i2c0;  // i2c0 and i2c1 are the default objects

    ESP_ERROR_CHECK(myI2C.begin(GPIO_NUM_21, GPIO_NUM_22, GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE, I2C_BUS_SPEED));
    myI2C.setTimeout(10);
    printf("I2C initialized \n");

    ESP_ERROR_CHECK(myI2C.testConnection(MAX31790_ADDRESS));
    printf("I2C connection tested \n");

    FanControl* fanControl = new FanControl(&myI2C, MAX31790_ADDRESS);
    ESP_ERROR_CHECK(fanControl->init());
    printf("FanControl Initialized \n");

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
    //
    //for (int i = 10; i >= 0; i--) {
    //    printf("Restarting in %d seconds...\n", i);
    //    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //}
    //printf("Restarting now.\n");
    fflush(stdout);
    //esp_restart();
}
