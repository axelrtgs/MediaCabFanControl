#pragma once

#ifndef WifiProvMgr_h
#define WifiProvMgr_h

#include <dispatcher.h>
#include <esp_event.h>
#include <stdint.h>

#include <functional>

namespace {
const int WIFI_CONNECTED_EVENT = BIT0;
}

namespace wifi_prov_mgr {
enum class WifiAssociationState : uint8_t {
  CONNECTED = 0,
  DISCONNECTED = 1,
  CONNECTING = 2
};

typedef struct WifiConnectionState {
  WifiAssociationState wifiState = WifiAssociationState::DISCONNECTED;
  std::string ipAddr = "N/A";
} WifiConnectionState_t;

using wificallback_t = std::function<void(WifiAssociationState)>;
using WifiConnectionStateCB = std::function<void(WifiConnectionState_t)>;
class wifi_prov_mgr {
 public:
  wifi_prov_mgr() = default;
  void init(const char *pop);
  void registerCallback(WifiConnectionStateCB callback);

  std::string getIpAddrStr() const;
  WifiConnectionState getWifiState();

 private:
  void wifiStateChanged(WifiAssociationState state);

  Utilities::Dispatcher<WifiConnectionState_t> mWifiStateNotifier;
  // Need to store so doesnt go out of scope in C land
  wificallback_t mWifiCallback;
};
}  // namespace wifi_prov_mgr
#endif
