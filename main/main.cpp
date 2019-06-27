//#include <stdio.h>
//#include <stdlib.h>
//#include <stdint.h>
#include <memory>

#include "FanControl.h"
#include "PIDEnhanced.h"

#include "blufi.h"

extern "C"
{
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    //#include "freertos/event_groups.h"
    //#include "esp_system.h"
    //#include "esp_wifi.h"
    //#include "esp_event_loop.h"
    //#include "esp_spi_flash.h"
    #include "driver/gpio.h"
    //#include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_task_wdt.h"
    #include "nvs_flash.h"

    #include "homekit.h"
}

namespace
{
    const uint32_t I2C_BUS_SPEED = 400000;
    const uint8_t MAX31790_ADDRESS = 0x2F;
}

std::shared_ptr<blufi::blufi> mblufi(new blufi::blufi());
bool connected = false;
fan_kit fanKit;

extern "C" {
    void setupApp();
    void blufi_callback(blufi::WifiConnectionState state);

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

        mblufi->init();
        mblufi->registerCallback(blufi_callback);
    }

    void valuesTask(void *_args)
    {
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
                .Kp = 10,
                .Ki = 1,
                .Kd = 0
            };

        PIDEnhanced::PID_TUNING aggressive_tune
            {
                .Kp = 50,
                .Ki = 10,
                .Kd = 0
            };

        PIDEnhanced*   _PID;
        double   _PID_output;
        const int tolerance = false;

        _PID = new PIDEnhanced(tolerance, PWM_MIN, PWM_MAX, conservative_tune, aggressive_tune);
        while(1)
        {

            std::vector<double> inputVector = {fanKit.cur_temp};
            printf("Mode: %d CurTemp: %f, TargTemp: %f\n", fanKit.mode, fanKit.cur_temp, fanKit.target_temp);
            std::string PID_Profile = _PID->computeAvgOfVector(inputVector, fanKit.target_temp, &_PID_output);

            printf("PID Profile: %s \n", PID_Profile.c_str());

            printf("PID Out: %f \n", _PID_output);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    void blufi_callback(blufi::WifiConnectionState state)
    {
        if (state.wifiState == blufi::WifiAssociationState::CONNECTED && !connected) {
            printf("Wifi connected Starting homekit\n");
            homekit_init(&fanKit);
            connected = true;
            //setupApp();
            xTaskCreate(valuesTask, "Values", 2048, NULL, 2, NULL);
        } else if (state.wifiState == blufi::WifiAssociationState::DISCONNECTED && connected) {
            printf("Wifi disconnected\n");
            connected = false;
        }
    }

    //void setupApp()
    //{
    //    uint16_t pwmDuty[NUM_PWM];
    //    fanControl->getPWMDutyComplete(pwmDuty);
    //
    //    for (auto it = std::begin(pwmDuty); it != std::end(pwmDuty); ++it) {
    //        printf("PWM duty is: %d\n", *it);
    //    }
    //
    //    uint16_t pwmTarget[NUM_PWM];
    //    fanControl->getPWMTargetComplete(pwmTarget);
    //
    //    for (auto it = std::begin(pwmTarget); it != std::end(pwmTarget); ++it) {
    //        printf("PWM target is: %d\n", *it);
    //    }
    //
    //    uint16_t pwmNewTarget[] = { 300, 511, 450, 225 };
    //    fanControl->setPWMTargetComplete(pwmNewTarget);
    //
    //    fanControl->getPWMTargetComplete(pwmTarget);
    //
    //    for (auto it = std::begin(pwmTarget); it != std::end(pwmTarget); ++it) {
    //        printf("New PWM target is: %d\n", *it);
    //    }
    //
    //    fflush(stdout);
    //}
}
