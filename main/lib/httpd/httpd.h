#pragma once

#ifndef HTTPD_H
#define HTTPD_H

#include "ota.h"
#include <esp_err.h>
#include <esp_http_server.h>

namespace httpd {
/* Event callback types */
using httpd_on_ota_completed_cb_t = std::function<void(OTA::ERR)>;

class httpd {
public:
  httpd() = default;
  esp_err_t init(void);
  void set_on_ota_completed_cb(httpd_on_ota_completed_cb_t cb);

private:
  esp_err_t register_static_routes(httpd_handle_t server);
  esp_err_t register_ota_routes(httpd_handle_t server);

  httpd_handle_t server = NULL;
};
} // namespace httpd
#endif
