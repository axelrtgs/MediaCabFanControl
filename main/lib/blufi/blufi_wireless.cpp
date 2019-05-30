#include "blufi_wireless.h"

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

esp_blufi_callbacks_t blufi_callbacks = {
    .event_cb = blufi_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    wifi_mode_t mode;

    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            BLUFI_INFO("%s STA Start\n", __func__);
            esp_wifi_connect();
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
            wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);
            if (!ap_list) {
                BLUFI_ERROR("%s malloc error, ap_list is NULL\n", __func__);
                break;
            }
            ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, ap_list));
            esp_blufi_ap_record_t * blufi_ap_list = (esp_blufi_ap_record_t *)malloc(apCount * sizeof(esp_blufi_ap_record_t));
            if (!blufi_ap_list) {
                if (ap_list) {
                    free(ap_list);
                }
                BLUFI_ERROR("%s malloc error, blufi_ap_list is NULL\n", __func__);
                break;
            }
            for (int i = 0; i < apCount; ++i)
            {
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

void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param)
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
            ESP_ERROR_CHECK( esp_wifi_set_mode(param->wifi_mode.op_mode) );
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

            if (gl_sta_connected ) {
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
            strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid, param->sta_ssid.ssid_len);
            sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
            esp_wifi_set_config(WIFI_IF_STA, &sta_config);
            BLUFI_INFO("%s Recv STA SSID %s\n", __func__, sta_config.sta.ssid);
            break;
        case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
            strncpy((char *)sta_config.sta.password, (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
            sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
            esp_wifi_set_config(WIFI_IF_STA, &sta_config);
            BLUFI_INFO("%s Recv STA PASSWORD %s\n", __func__, sta_config.sta.password);
            break;
        case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
            strncpy((char *)ap_config.ap.ssid, (char *)param->softap_ssid.ssid, param->softap_ssid.ssid_len);
            ap_config.ap.ssid[param->softap_ssid.ssid_len] = '\0';
            ap_config.ap.ssid_len = param->softap_ssid.ssid_len;
            esp_wifi_set_config(WIFI_IF_AP, &ap_config);
            BLUFI_INFO("%s Recv SOFTAP SSID %s, ssid len %d\n", __func__, ap_config.ap.ssid, ap_config.ap.ssid_len);
            break;
        case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
            strncpy((char *)ap_config.ap.password, (char *)param->softap_passwd.passwd, param->softap_passwd.passwd_len);
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
        case ESP_BLUFI_EVENT_GET_WIFI_LIST:{
            wifi_scan_config_t scanConf = {
                .ssid = NULL,
                .bssid = NULL,
                .channel = 0,
                .show_hidden = false,
                .scan_type = WIFI_SCAN_TYPE_ACTIVE,
                .scan_time = {},
            };
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

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&blufi_adv_params);
            break;
        default:
            break;
    }
}