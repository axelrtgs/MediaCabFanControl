#pragma once

#ifndef WifiProvMgr_h
#define WifiProvMgr_h

#include <dispatcher.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <stdint.h>

#include <functional>

namespace {
const int WIFI_CONNECTED_EVENT = BIT0;
const size_t MAX_HOSTNAME_LENGTH = 25;
} // namespace

namespace wifi_prov_mgr {
enum class WifiAssociationState : uint8_t { CONNECTED = 0, DISCONNECTED = 1, CONNECTING = 2 };

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
  static void set_hostname(const char *prefix);
  static const char *get_hostname();

  std::string getIpAddrStr() const;
  WifiConnectionState getWifiState();

private:
  void wifiStateChanged(WifiAssociationState state);

  inline static char device_hostname[MAX_HOSTNAME_LENGTH];
  Utilities::Dispatcher<WifiConnectionState_t> mWifiStateNotifier;
  wificallback_t mWifiCallback;
};
} // namespace wifi_prov_mgr
#endif
