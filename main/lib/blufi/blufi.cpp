#include "blufi.h"

int checkOverrideGpio() {
    if (BLUFI_OVERRIDE_GPIO != GPIO_NUM_NC) {
        gpio_pad_select_gpio(BLUFI_OVERRIDE_GPIO);
        gpio_set_direction(BLUFI_OVERRIDE_GPIO, GPIO_MODE_INPUT);
        gpio_set_pull_mode(BLUFI_OVERRIDE_GPIO, GPIO_PULLDOWN_ONLY);
        return !gpio_get_level(BLUFI_OVERRIDE_GPIO);
    }
    else
        return 0;
}

esp_err_t  blufi_init() {
    esp_err_t ret;

    wifi_config_t sta_stored_config;
    bool sta_config_valid = true;

    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_FLASH) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    ESP_ERROR_CHECK(esp_wifi_get_config(ESP_IF_WIFI_STA, &sta_stored_config));

    if (strlen((char *)sta_stored_config.sta.ssid) == 0 || strlen((char *)sta_stored_config.sta.password) == 0) {
        BLUFI_WARN("%s Stored Wifi config invalid", __func__);
        sta_config_valid = false;
    }

    if (!checkOverrideGpio() && sta_config_valid) {
        int retries = 5;
        while (retries--) {
            BLUFI_INFO("%s Trying to connect to wifi ssid: %s using password: %s\n", __func__, (char *)sta_stored_config.sta.ssid, (char *)sta_stored_config.sta.password);
            ESP_ERROR_CHECK(esp_wifi_connect());
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if (gl_sta_connected)
                break;
            BLUFI_INFO("%s Failed to connect to wifi. %d retries left\n", __func__, retries);
        }
        if (retries > 0) {
            BLUFI_INFO("%s Wifi configured disabling bluetooth onboarding.\n", __func__);
            return ESP_OK;
        } else {
            BLUFI_WARN("%s Wifi NOT configured enabling bluetooth onboarding.\n", __func__);
            ESP_ERROR_CHECK(esp_wifi_disconnect());
        }
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        BLUFI_ERROR("%s initialize bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        BLUFI_ERROR("%s enable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    BLUFI_INFO("BD ADDR: " ESP_BD_ADDR_STR "\n", ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));

    BLUFI_INFO("BLUFI VERSION %04x\n", esp_blufi_get_version());

    ret = blufi_security_init();
    if(ret){
        BLUFI_ERROR("%s blufi security init failed, error code = %x\n", __func__, ret);
        return ret;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if(ret){
        BLUFI_ERROR("%s gap register failed, error code = %x\n", __func__, ret);
        return ret;
    }

    ret = esp_blufi_register_callbacks(&blufi_callbacks);
    if(ret){
        BLUFI_ERROR("%s blufi register failed, error code = %x\n", __func__, ret);
        return ret;
    }

    ret = esp_blufi_profile_init();
    if(ret){
        BLUFI_ERROR("%s blufi profile init failed, error code = %x\n", __func__, ret);
        return ret;
    }
    return ESP_OK;
}
