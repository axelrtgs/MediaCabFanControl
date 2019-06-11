#include "blufi.h"
#include <iomanip>

extern "C" {
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event_loop.h"

#include "esp_wifi.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_blufi_api.h"
#include "esp_gap_ble_api.h"

#include <lwip/sockets.h>

namespace blufi
{
    /* FreeRTOS event group to signal when we are connected & ready to make a request */
    EventGroupHandle_t wifi_event_group;
    wifi_config_t sta_config;
    wifi_config_t ap_config;

    /* store the station info for send back to phone */
    bool gl_sta_connected = false;
    uint8_t gl_sta_bssid[6];
    uint8_t gl_sta_ssid[32];
    int gl_sta_ssid_len;

    /* connect infor*/
    uint8_t server_if;
    uint16_t conn_id;

    struct blufi_security *blufiSecurity;

    uint8_t blufi_service_uuid128[32] = {
        /* LSB <--------------------------------------------------------------------------------> MSB */
        //first uuid, 16bit, [12],[13] is the value
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
     };

    //uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
    esp_ble_adv_data_t blufi_adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
        .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data =  nullptr,
        .service_data_len = 0,
        .p_service_data = nullptr,
        .service_uuid_len = 16,
        .p_service_uuid = blufi_service_uuid128,
        .flag = 0x6,
    };

    esp_ble_adv_params_t blufi_adv_params = {
        0x100,
        0x100,
        ADV_TYPE_IND,
        BLE_ADDR_TYPE_PUBLIC,
        {},
        BLE_ADDR_TYPE_PUBLIC,
        ADV_CHNL_ALL,
        ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    esp_err_t blufi_security_init(void)
    {
        blufiSecurity = (struct blufi_security*) malloc(sizeof(struct blufi_security));
        if (blufiSecurity == NULL) {
            return ESP_FAIL;
        }

        memset(blufiSecurity, 0x0, sizeof(struct blufi_security));

        mbedtls_dhm_init(&blufiSecurity->dhm);
        mbedtls_aes_init(&blufiSecurity->aes);

        memset(blufiSecurity->iv, 0x0, 16);
        return ESP_OK;
    }

    void blufi_security_deinit(void)
    {
        if (blufiSecurity == NULL) {
            return;
        }
        if (blufiSecurity->dh_param) {
            free(blufiSecurity->dh_param);
            blufiSecurity->dh_param = NULL;
        }
        mbedtls_dhm_free(&blufiSecurity->dhm);
        mbedtls_aes_free(&blufiSecurity->aes);

        memset(blufiSecurity, 0x0, sizeof(struct blufi_security));

        free(blufiSecurity);
        blufiSecurity = NULL;
    }

    esp_err_t wifi_event_handler(void* ctx, system_event_t* event)
    {
        wifi_mode_t mode;
        auto& callback = *reinterpret_cast<wificallback_t*>(ctx);

        switch (event->event_id) {
            case SYSTEM_EVENT_STA_START:
                BLUFI_INFO("%s STA Start\n", __func__);
                esp_wifi_connect();
                callback(WifiAssociationState::CONNECTING);
                break;
            case SYSTEM_EVENT_STA_GOT_IP: {
                esp_blufi_extra_info_t info;

                BLUFI_INFO("%s STA Got IP: %s\n", __func__, ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
                xEventGroupSetBits(wifi_event_group, WIFI_GOT_IP_BIT);
                esp_wifi_get_mode(&mode);

                memset(&info, 0, sizeof(esp_blufi_extra_info_t));
                memcpy(info.sta_bssid, gl_sta_bssid, 6);
                info.sta_bssid_set = true;
                info.sta_ssid = gl_sta_ssid;
                info.sta_ssid_len = gl_sta_ssid_len;
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
                callback(WifiAssociationState::CONNECTED);
                break;
            }
            case SYSTEM_EVENT_AP_STA_GOT_IP6: {
                esp_blufi_extra_info_t info;

                BLUFI_INFO("%s STA Got IP6: %s\n", __func__, ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip));
                xEventGroupSetBits(wifi_event_group, WIFI_GOT_IP6_BIT);
                esp_wifi_get_mode(&mode);

                memset(&info, 0, sizeof(esp_blufi_extra_info_t));
                memcpy(info.sta_bssid, gl_sta_bssid, 6);
                info.sta_bssid_set = true;
                info.sta_ssid = gl_sta_ssid;
                info.sta_ssid_len = gl_sta_ssid_len;
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
                callback(WifiAssociationState::CONNECTED);
                break;
            }
            case SYSTEM_EVENT_STA_CONNECTED:
                gl_sta_connected = true;
                BLUFI_INFO("%s STA Connected\n", __func__);
                memcpy(gl_sta_bssid, event->event_info.connected.bssid, 6);
                memcpy(gl_sta_ssid, event->event_info.connected.ssid, event->event_info.connected.ssid_len);
                gl_sta_ssid_len = event->event_info.connected.ssid_len;
                tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
                break;
            case SYSTEM_EVENT_STA_DISCONNECTED:
                /* This is a workaround as ESP32 WiFi libs don't currently
                   auto-reassociate. */
                BLUFI_INFO("%s STA Disconnected\n", __func__);
                gl_sta_connected = false;
                memset(gl_sta_ssid, 0, 32);
                memset(gl_sta_bssid, 0, 6);
                gl_sta_ssid_len = 0;
                esp_wifi_disconnect();
                esp_wifi_connect();
                xEventGroupClearBits(wifi_event_group, WIFI_GOT_IP_BIT);
                xEventGroupClearBits(wifi_event_group, WIFI_GOT_IP6_BIT);
                callback(WifiAssociationState::CONNECTING);
                break;
            case SYSTEM_EVENT_AP_START:
                esp_wifi_get_mode(&mode);

                /* TODO: get config or information of softap, then set to report extra_info */
                if (gl_sta_connected) {
                    esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, NULL);
                } else {
                    esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
                }
                break;
            case SYSTEM_EVENT_SCAN_DONE: {
                uint16_t apCount = 0;
                esp_wifi_scan_get_ap_num(&apCount);
                if (apCount == 0) {
                    BLUFI_INFO("%s Nothing AP found\n", __func__);
                    break;
                }
                wifi_ap_record_t* ap_list = (wifi_ap_record_t*) malloc(sizeof(wifi_ap_record_t) * apCount);
                if (!ap_list) {
                    BLUFI_ERROR("%s malloc error, ap_list is NULL\n", __func__);
                    break;
                }
                ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, ap_list));
                esp_blufi_ap_record_t* blufi_ap_list = (esp_blufi_ap_record_t*) malloc(apCount * sizeof(esp_blufi_ap_record_t));
                if (!blufi_ap_list) {
                    if (ap_list) {
                        free(ap_list);
                    }
                    BLUFI_ERROR("%s malloc error, blufi_ap_list is NULL\n", __func__);
                    break;
                }
                for (int i = 0; i < apCount; ++i) {
                    blufi_ap_list[i].rssi = ap_list[i].rssi;
                    memcpy(blufi_ap_list[i].ssid, ap_list[i].ssid, sizeof(ap_list[i].ssid));
                }
                esp_blufi_send_wifi_list(apCount, blufi_ap_list);
                esp_wifi_scan_stop();
                free(ap_list);
                free(blufi_ap_list);
                break;
            }
            default:
                break;
        }
        return ESP_OK;
    }

    void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&blufi_adv_params);
            break;
        default:
            break;
    }
}

    void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t* param)
    {
        /* actually, should post to blufi_task handle the procedure,
         * now, as a demo, we do simply */
        switch (event) {
            case ESP_BLUFI_EVENT_INIT_FINISH:
                BLUFI_INFO("%s BLUFI init finish\n", __func__);
                esp_ble_gap_set_device_name(BLUFI_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&blufi_adv_data);
                break;
            case ESP_BLUFI_EVENT_DEINIT_FINISH:
                BLUFI_INFO("%s BLUFI init finish\n", __func__);
                break;
            case ESP_BLUFI_EVENT_BLE_CONNECT:
                BLUFI_INFO("%s BLUFI ble connect\n", __func__);
                server_if = param->connect.server_if;
                conn_id = param->connect.conn_id;
                esp_ble_gap_stop_advertising();
                blufi_security_init();
                break;
            case ESP_BLUFI_EVENT_BLE_DISCONNECT:
                BLUFI_INFO("%s BLUFI ble disconnect\n", __func__);
                blufi_security_deinit();
                esp_ble_gap_start_advertising(&blufi_adv_params);
                break;
            case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
                BLUFI_INFO("%s BLUFI Set WIFI opmode %d\n", __func__, param->wifi_mode.op_mode);
                ESP_ERROR_CHECK(esp_wifi_set_mode(param->wifi_mode.op_mode));
                break;
            case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
                BLUFI_INFO("%s BLUFI request wifi connect to AP\n", __func__);
                /* there is no wifi callback when the device has already connected to this wifi
                so disconnect wifi before connection.
                */
                esp_wifi_disconnect();
                esp_wifi_connect();
                break;
            case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
                BLUFI_INFO("%s BLUFI requset wifi disconnect from AP\n", __func__);
                esp_wifi_disconnect();
                break;
            case ESP_BLUFI_EVENT_REPORT_ERROR:
                BLUFI_ERROR("%s BLUFI report error, error code %d\n", __func__, param->report_error.state);
                esp_blufi_send_error_info(param->report_error.state);
                break;
            case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
                wifi_mode_t mode;
                esp_blufi_extra_info_t info;

                esp_wifi_get_mode(&mode);

                if (gl_sta_connected) {
                    memset(&info, 0, sizeof(esp_blufi_extra_info_t));
                    memcpy(info.sta_bssid, gl_sta_bssid, 6);
                    info.sta_bssid_set = true;
                    info.sta_ssid = gl_sta_ssid;
                    info.sta_ssid_len = gl_sta_ssid_len;
                    esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
                } else {
                    esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
                }
                BLUFI_INFO("%s BLUFI get wifi status from AP\n", __func__);

                break;
            }
            case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
                BLUFI_INFO("%s BLUFI close a gatt connection\n", __func__);
                esp_blufi_close(server_if, conn_id);
                break;
            case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
                /* TODO */
                break;
            case ESP_BLUFI_EVENT_RECV_STA_BSSID:
                memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
                sta_config.sta.bssid_set = 1;
                esp_wifi_set_config(WIFI_IF_STA, &sta_config);
                BLUFI_INFO("%s Recv STA BSSID %s\n", sta_config.sta.ssid, __func__);
                break;
            case ESP_BLUFI_EVENT_RECV_STA_SSID:
                strncpy((char*) sta_config.sta.ssid, (char*) param->sta_ssid.ssid, param->sta_ssid.ssid_len);
                sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
                esp_wifi_set_config(WIFI_IF_STA, &sta_config);
                BLUFI_INFO("%s Recv STA SSID %s\n", __func__, sta_config.sta.ssid);
                break;
            case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
                strncpy((char*) sta_config.sta.password, (char*) param->sta_passwd.passwd, param->sta_passwd.passwd_len);
                sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
                esp_wifi_set_config(WIFI_IF_STA, &sta_config);
                BLUFI_INFO("%s Recv STA PASSWORD %s\n", __func__, sta_config.sta.password);
                break;
            case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
                strncpy((char*) ap_config.ap.ssid, (char*) param->softap_ssid.ssid, param->softap_ssid.ssid_len);
                ap_config.ap.ssid[param->softap_ssid.ssid_len] = '\0';
                ap_config.ap.ssid_len = param->softap_ssid.ssid_len;
                esp_wifi_set_config(WIFI_IF_AP, &ap_config);
                BLUFI_INFO("%s Recv SOFTAP SSID %s, ssid len %d\n", __func__, ap_config.ap.ssid, ap_config.ap.ssid_len);
                break;
            case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
                strncpy((char*) ap_config.ap.password, (char*) param->softap_passwd.passwd, param->softap_passwd.passwd_len);
                ap_config.ap.password[param->softap_passwd.passwd_len] = '\0';
                esp_wifi_set_config(WIFI_IF_AP, &ap_config);
                BLUFI_INFO("%s Recv SOFTAP PASSWORD %s\n", __func__, ap_config.ap.password);
                break;
            case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
                if (param->softap_max_conn_num.max_conn_num > 4) {
                    return;
                }
                ap_config.ap.max_connection = param->softap_max_conn_num.max_conn_num;
                esp_wifi_set_config(WIFI_IF_AP, &ap_config);
                BLUFI_INFO("%s Recv SOFTAP MAX CONN NUM %d\n", __func__, ap_config.ap.max_connection);
                break;
            case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
                if (param->softap_auth_mode.auth_mode >= WIFI_AUTH_MAX) {
                    return;
                }
                ap_config.ap.authmode = param->softap_auth_mode.auth_mode;
                esp_wifi_set_config(WIFI_IF_AP, &ap_config);
                BLUFI_INFO("%s Recv SOFTAP AUTH MODE %d\n", __func__, ap_config.ap.authmode);
                break;
            case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
                if (param->softap_channel.channel > 13) {
                    return;
                }
                ap_config.ap.channel = param->softap_channel.channel;
                esp_wifi_set_config(WIFI_IF_AP, &ap_config);
                BLUFI_INFO("%s Recv SOFTAP CHANNEL %d\n", __func__, ap_config.ap.channel);
                break;
            case ESP_BLUFI_EVENT_GET_WIFI_LIST: {
                wifi_scan_config_t scanConf =
                    {.ssid = NULL, .bssid = NULL, .channel = 0, .show_hidden = false, .scan_type = WIFI_SCAN_TYPE_ACTIVE, .scan_time = {},};
                ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
                break;
            }
            case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
                BLUFI_INFO("%s Recv Custom Data %d\n", __func__, param->custom_data.data_len);
                        esp_log_buffer_hex("Custom Data", param->custom_data.data, param->custom_data.data_len);
                break;
            case ESP_BLUFI_EVENT_RECV_USERNAME:
                /* Not handle currently */
                break;
            case ESP_BLUFI_EVENT_RECV_CA_CERT:
                /* Not handle currently */
                break;
            case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
                /* Not handle currently */
                break;
            case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
                /* Not handle currently */
                break;
            case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
                /* Not handle currently */
                break;;
            case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
                /* Not handle currently */
                break;
            default:
                break;
        }
    }

    void blufi_dh_negotiate_data_handler(uint8_t* data, int len, uint8_t** output_data, int* output_len, bool* need_free)
    {
        int ret;
        uint8_t type = data[0];

        if (blufiSecurity == NULL) {
            BLUFI_ERROR("%s BLUFI Security is not initialized\n", __func__);
            btc_blufi_report_error(ESP_BLUFI_INIT_SECURITY_ERROR);
            return;
        }

        switch (type) {
            case SEC_TYPE_DH_PARAM_LEN:
                blufiSecurity->dh_param_len = ((data[1] << 8) | data[2]);
                if (blufiSecurity->dh_param) {
                    free(blufiSecurity->dh_param);
                    blufiSecurity->dh_param = NULL;
                }
                blufiSecurity->dh_param = (uint8_t*) malloc(blufiSecurity->dh_param_len);
                if (blufiSecurity->dh_param == NULL) {
                    btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                    BLUFI_ERROR("%s, malloc failed\n", __func__);
                    return;
                }
                break;
            case SEC_TYPE_DH_PARAM_DATA: {
                if (blufiSecurity->dh_param == NULL) {
                    BLUFI_ERROR("%s, blufiSecurity->dh_param == NULL\n", __func__);
                    btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
                    return;
                }
                uint8_t* param = blufiSecurity->dh_param;
                memcpy(blufiSecurity->dh_param, &data[1], blufiSecurity->dh_param_len);
                ret = mbedtls_dhm_read_params(&blufiSecurity->dhm, &param, &param[blufiSecurity->dh_param_len]);
                if (ret) {
                    BLUFI_ERROR("%s read param failed %d\n", __func__, ret);
                    btc_blufi_report_error(ESP_BLUFI_READ_PARAM_ERROR);
                    return;
                }
                free(blufiSecurity->dh_param);
                blufiSecurity->dh_param = NULL;
                ret = mbedtls_dhm_make_public(&blufiSecurity->dhm,
                                              (int) mbedtls_mpi_size(&blufiSecurity->dhm.P),
                                              blufiSecurity->self_public_key,
                                              blufiSecurity->dhm.len,
                                              myrand,
                                              NULL);
                if (ret) {
                    BLUFI_ERROR("%s make public failed %d\n", __func__, ret);
                    btc_blufi_report_error(ESP_BLUFI_MAKE_PUBLIC_ERROR);
                    return;
                }

                mbedtls_dhm_calc_secret(&blufiSecurity->dhm,
                                        blufiSecurity->share_key,
                                        SHARE_KEY_BIT_LEN,
                                        &blufiSecurity->share_len,
                                        myrand,
                                        NULL);

                mbedtls_md5(blufiSecurity->share_key, blufiSecurity->share_len, blufiSecurity->psk);

                mbedtls_aes_setkey_enc(&blufiSecurity->aes, blufiSecurity->psk, 128);
                mbedtls_aes_setkey_dec(&blufiSecurity->aes, blufiSecurity->psk, 128);

                /* alloc output data */
                *output_data = &blufiSecurity->self_public_key[0];
                *output_len = blufiSecurity->dhm.len;
                *need_free = false;
                break;
            }
            case SEC_TYPE_DH_P:
                break;
            case SEC_TYPE_DH_G:
                break;
            case SEC_TYPE_DH_PUBLIC:
                break;
        }
    }

    int blufi_aes_encrypt(uint8_t iv8, uint8_t* crypt_data, int crypt_len)
    {
        int ret;
        size_t iv_offset = 0;
        uint8_t iv0[16];

        memcpy(iv0, blufiSecurity->iv, sizeof(blufiSecurity->iv));
        iv0[0] = iv8;   /* set iv8 as the iv0[0] */

        ret = mbedtls_aes_crypt_cfb128(&blufiSecurity->aes, MBEDTLS_AES_ENCRYPT, crypt_len, &iv_offset, iv0, crypt_data, crypt_data);
        if (ret) {
            return -1;
        }

        return crypt_len;
    }

    int blufi_aes_decrypt(uint8_t iv8, uint8_t* crypt_data, int crypt_len)
    {
        int ret;
        size_t iv_offset = 0;
        uint8_t iv0[16];

        memcpy(iv0, blufiSecurity->iv, sizeof(blufiSecurity->iv));
        iv0[0] = iv8;   /* set iv8 as the iv0[0] */

        ret = mbedtls_aes_crypt_cfb128(&blufiSecurity->aes, MBEDTLS_AES_DECRYPT, crypt_len, &iv_offset, iv0, crypt_data, crypt_data);
        if (ret) {
            return -1;
        }

        return crypt_len;
    }

    uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t* data, int len)
    {
        /* This iv8 ignore, not used */
        return crc16_be(0, data, len);
    }

    esp_blufi_callbacks_t blufi_callbacks = {
        .event_cb = blufi_event_callback,
        .negotiate_data_handler = blufi_dh_negotiate_data_handler,
        .encrypt_func = blufi_aes_encrypt,
        .decrypt_func = blufi_aes_decrypt,
        .checksum_func = blufi_crc_checksum,
    };

    void blufi::init()
{
    esp_err_t ret;

    wifi_config_t sta_stored_config;
    bool sta_config_valid = true;

    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    using namespace std::placeholders;
    mWifiCallback = std::bind(&blufi::wifiStateChanged, this, _1);
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, &mWifiCallback));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_get_config(ESP_IF_WIFI_STA, &sta_stored_config));

    if (strlen((char*) sta_stored_config.sta.ssid) == 0 || strlen((char*) sta_stored_config.sta.password) == 0) {
        BLUFI_WARN("%s Stored Wifi config invalid", __func__);
        sta_config_valid = false;
    }

    if (sta_config_valid) {
        int retries = 5;
        while (retries--) {
            BLUFI_INFO("%s Trying to connect to wifi ssid: %s using password: %s\n",
                       __func__,
                       (char*) sta_stored_config.sta.ssid,
                       (char*) sta_stored_config.sta.password);
            ESP_ERROR_CHECK(esp_wifi_connect());
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if (gl_sta_connected)
                break;
            BLUFI_INFO("%s Failed to connect to wifi. %d retries left\n", __func__, retries);
        }
        if (retries > 0) {
            BLUFI_INFO("%s Wifi configured disabling bluetooth onboarding.\n", __func__);
            return;
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
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        BLUFI_ERROR("%s enable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        BLUFI_ERROR("%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    BLUFI_INFO("BD ADDR: "
                   ESP_BD_ADDR_STR
                   "\n", ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));

    BLUFI_INFO("BLUFI VERSION %04x\n", esp_blufi_get_version());

    ret = blufi_security_init();
    if (ret) {
        BLUFI_ERROR("%s blufi security init failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        BLUFI_ERROR("%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_blufi_register_callbacks(&blufi_callbacks);
    if (ret) {
        BLUFI_ERROR("%s blufi register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_blufi_profile_init();
    if (ret) {
        BLUFI_ERROR("%s blufi profile init failed, error code = %x\n", __func__, ret);
        return;
    }
}

    void blufi::registerCallback(WifiConnectionStateCB callback)
    {
        mWifiStateNotifier.addCB(callback);
    }

    std::string blufi::getIpAddrStr() const
    {
        tcpip_adapter_ip_info_t ipInfo;

        tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);
        char ipAddrStr[30];
        inet_ntop(AF_INET, &ipInfo.ip.addr, ipAddrStr, sizeof(ipAddrStr));
        return std::string(ipAddrStr);
    }

    void blufi::wifiStateChanged(WifiAssociationState state)
    {
        WifiConnectionState wifiConnState;
        if (state == WifiAssociationState::CONNECTED) {
            wifiConnState.ipAddr = getIpAddrStr();
        }
        wifiConnState.wifiState = state;
        mWifiStateNotifier.broadcast(wifiConnState);
    }
}
}