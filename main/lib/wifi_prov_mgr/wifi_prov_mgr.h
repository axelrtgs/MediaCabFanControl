#pragma once

#ifndef WifiProvMgr_h
#define WifiProvMgr_h

#include <dispatcher.h>

#include <functional>

namespace {
const int WIFI_CONNECTED_EVENT = BIT0;
}

namespace wifi_prov_mgr {
enum class WifiAssociationState : int {
  CONNECTED = 0,
  DISCONNECTED = 1,
  CONNECTING = 2
};

struct WifiConnectionState {
  WifiAssociationState wifiState = WifiAssociationState::DISCONNECTED;
  std::string ipAddr = "N/A";
};

using wificallback_t = std::function<void(WifiAssociationState)>;
using WifiConnectionStateCB = std::function<void(WifiConnectionState)>;
class wifi_prov_mgr {
 public:
  wifi_prov_mgr() = default;
  void init(const char *pop);
  void registerCallback(WifiConnectionStateCB callback);

  std::string getIpAddrStr() const;
  WifiConnectionState getWifiState();

 private:
  void wifiStateChanged(WifiAssociationState state);

  Dispatcher<WifiConnectionState> mWifiStateNotifier;
  // Need to store so doesnt go out of scope in C land
  wificallback_t mWifiCallback;
};
}  // namespace wifi_prov_mgr
#endif
