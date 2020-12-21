#include <stdio.h>
#include <string.h>
#include <dispatcher.h>
#include <functional>
#include <lwip/sockets.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>

namespace
{
  const int WIFI_CONNECTED_EVENT = BIT0;
}

namespace wifi_prov_mgr
{
  enum class WifiAssociationState: int
  {
      CONNECTED = 0,
      DISCONNECTED = 1,
      CONNECTING = 2
  };

  struct WifiConnectionState
  {
      WifiAssociationState wifiState = WifiAssociationState::DISCONNECTED;
      std::string ipAddr = "N/A";
  };

  using wificallback_t = std::function<void(wifi_prov_mgr::WifiAssociationState)>;
  using WifiConnectionStateCB = std::function<void(wifi_prov_mgr::WifiConnectionState)>;
  class wifi_prov_mgr
  {
  public:
      wifi_prov_mgr() = default;
      void init();
      void registerCallback(WifiConnectionStateCB callback);

      std::string getIpAddrStr() const;
      WifiConnectionState getWifiState();


  private:
      void wifiStateChanged(WifiAssociationState state);

      Dispatcher<WifiConnectionState> mWifiStateNotifier;
      wificallback_t mWifiCallback; // Need to store so doesnt go out of scope in C land
  };
}
