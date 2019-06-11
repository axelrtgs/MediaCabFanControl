//#define CONFIG_I2CBUS_LOG_READWRITES
//#define CONFIG_I2CBUS_LOG_RW_LEVEL_INFO

//#include <stdio.h>
//#include <stdlib.h>
//#include <stdint.h>
#include <memory>

//#include "FanControl.h"
//#include "PIDEnhanced.h"

#include "blufi.h"

extern "C"
{
    //#include "freertos/event_groups.h"
    //#include "esp_system.h"
    //#include "esp_wifi.h"
    //#include "esp_event_loop.h"
    //#include "esp_spi_flash.h"
    //#include "driver/gpio.h"
    //#include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_task_wdt.h"
    #include "nvs_flash.h"

    #include "homekit.h"
}



//namespace
//{
//const uint32_t I2C_BUS_SPEED = 400000;
//const uint8_t MAX31790_ADDRESS = 0x2F;
//}
std::shared_ptr<blufi::blufi> mblufi(new blufi::blufi());
bool connected = false;

extern "C" {
    void setupApp();

    void app_main()
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

        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            printf("No free pages erasing flash");
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        setupApp();
    }

    void blufi_callback(blufi::WifiConnectionState state)
    {
        if (state.wifiState == blufi::WifiAssociationState::CONNECTED && !connected) {
            printf("Wifi connected Starting homekit\n");
            homekit_init();
            connected = true;
        } else if (state.wifiState == blufi::WifiAssociationState::DISCONNECTED && connected) {
            printf("Wifi disconnected\n");
            connected = false;
        }
    }

    void setupApp()
    {
        //esp_err_t ret;
        mblufi->init();
        mblufi->registerCallback(blufi_callback);

        //printf("Starting FanControl \n");
        //I2C_t& myI2C = i2c0;  // i2c0 and i2c1 are the default objects
        //
        //ESP_ERROR_CHECK(myI2C.begin(GPIO_NUM_21, GPIO_NUM_22, GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE, I2C_BUS_SPEED));
        //myI2C.setTimeout(10);
        //printf("I2C initialized \n");
        //
        //ESP_ERROR_CHECK(myI2C.testConnection(MAX31790_ADDRESS));
        //printf("I2C connection tested \n");
        //
        //FanControl* fanControl = new FanControl(&myI2C, MAX31790_ADDRESS);
        //ESP_ERROR_CHECK(fanControl->init());
        //printf("FanControl Initialized \n");
        //
        //PIDEnhanced::PID_TUNING conservative_tune
        //{
        //    .Kp = -10.0,
        //    .Ki = 0.0,
        //    .Kd = 0.0
        //};
        //
        //PIDEnhanced::PID_TUNING aggressive_tune
        //{
        //    .Kp = -10.0,
        //    .Ki = 0.0,
        //    .Kd = 0.0
        //};
        //PIDEnhanced*   _PID;
        //std::vector<double> inputVector = {50.0, 50.0};
        //double   _PID_target = 50.0;
        //double   _PID_output;
        //
        //_PID = new PIDEnhanced(10, 0, 511, conservative_tune, aggressive_tune);
        //std::string PID_Profile = _PID->computeAvgOfVector(inputVector, _PID_target, &_PID_output);
        //
        //printf("PID Profile: %s \n", PID_Profile.c_str());
        //
        //printf("PID Out: %f \n", _PID_output);

        //fflush(stdout);
    }
}
