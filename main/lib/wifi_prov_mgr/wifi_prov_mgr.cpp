#include "wifi_prov_mgr.h"

namespace wifi_prov_mgr
{
  static EventGroupHandle_t wifi_event_group;
  static const char *TAG = "wifi_prov_mgr";

  static void event_handler(void* arg, esp_event_base_t event_base,
                            int event_id, void* event_data)
  {
    auto& callback = *reinterpret_cast<wificallback_t*>(arg);
    if (event_base == WIFI_PROV_EVENT) {
      switch (event_id) {
        case WIFI_PROV_START:
          ESP_LOGI(TAG, "Provisioning started");
          break;
        case WIFI_PROV_CRED_RECV: {
          wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
          ESP_LOGD(TAG, "Received Wi-Fi credentials"
                  "\n\tSSID     : %s\n\tPassword : %s",
                  (const char *) wifi_sta_cfg->ssid,
                  (const char *) wifi_sta_cfg->password);
          break;
        }
        case WIFI_PROV_CRED_FAIL: {
          wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
          ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                  "\n\tPlease reset to factory and retry provisioning",
                  (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
                  "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
          break;
        }
        case WIFI_PROV_CRED_SUCCESS:
          ESP_LOGI(TAG, "Provisioning successful");
          break;
        case WIFI_PROV_END:
          ESP_LOGD(TAG, "De-initialize manager");
          wifi_prov_mgr_deinit();
          break;
        default:
          break;
      }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
      ESP_LOGI(TAG, "Connecting to the AP...");
      esp_wifi_connect();
      callback(WifiAssociationState::CONNECTING);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
      ESP_LOGI(TAG, "Connected waiting for IP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

      ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
      xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
      callback(WifiAssociationState::CONNECTED);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
      ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
      esp_wifi_disconnect();
      xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_EVENT);
      esp_wifi_connect();
      callback(WifiAssociationState::CONNECTING);
    }
  }

  static void get_device_service_name(char *service_name, size_t max)
  {
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X",
            ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
  }

  void wifi_prov_mgr::init(const char *pop)
  {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_event_group = xEventGroupCreate();

    using namespace std::placeholders;
    mWifiCallback = std::bind(&wifi_prov_mgr::wifiStateChanged, this, _1);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, &mWifiCallback));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, &mWifiCallback));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, &mWifiCallback));

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };

    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
      ESP_LOGI(TAG, "Starting provisioning");

      char service_name[12];
      get_device_service_name(service_name, sizeof(service_name));

      wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
      const char *service_key = NULL;

      uint8_t custom_service_uuid[] = {
          /* LSB <---------------------------------------
          * ---------------------------------------> MSB */
          0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
          0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
      };
      wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

      ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, pop, service_name, service_key));
    } else {
      ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

      wifi_prov_mgr_deinit();
      ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
      ESP_ERROR_CHECK(esp_wifi_start());
    }
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, portMAX_DELAY);
  }

  void wifi_prov_mgr::registerCallback(WifiConnectionStateCB callback)
  {
    mWifiStateNotifier.addCB(callback);
    ESP_LOGD(TAG, "Callback added");
  }

  std::string wifi_prov_mgr::getIpAddrStr() const
  {
    tcpip_adapter_ip_info_t ipInfo;

    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);
    char ipAddrStr[30];
    inet_ntop(AF_INET, &ipInfo.ip.addr, ipAddrStr, sizeof(ipAddrStr));
    return std::string(ipAddrStr);
  }

  void wifi_prov_mgr::wifiStateChanged(WifiAssociationState state)
  {
    ESP_LOGD(TAG, "wifiStateChanged: %d", (int)state);
    WifiConnectionState wifiConnState;
    if (state == WifiAssociationState::CONNECTED) {
      wifiConnState.ipAddr = getIpAddrStr();
    }
    wifiConnState.wifiState = state;
    mWifiStateNotifier.broadcast(wifiConnState);
  }
}
