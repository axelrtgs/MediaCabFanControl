#pragma once

#ifndef Utilities_h
#define Utilities_h

#include <esp_log.h>
#include <lwip/netdb.h>

#include <algorithm>

namespace Utilities {
template <typename T> T clamp_val(const T &Value, const T &Min, const T &Max) {
  return std::max(Min, std::min(Value, Max));
}

inline const char *resolve_host(const char *hostname) {
  static char buf[64];
  struct hostent *host;
  const char *TAG = "Resolve";

  if (!(host = gethostbyname(hostname)) || host->h_addr_list[0] == NULL) {
    ESP_LOGE(TAG, "Failed resolving %s", hostname);
    return hostname;
  }

  sprintf(buf, "%s", inet_ntoa(*(struct ip4_addr *)host->h_addr_list[0]));
  ESP_LOGD(TAG, "DNS resolved %s to %s", hostname, buf);
  return buf;
}
} // namespace Utilities
#endif
