#include "httpd.h"
#include "httpd_static_files.h"
#include "ota.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

namespace httpd {
static httpd_on_ota_completed_cb_t on_ota_completed_cb;
static const char *TAG = "httpd";

static void ota_delayed_reset_timer_cb(TimerHandle_t xTimer) {
  if (!on_ota_completed_cb)
    return;

  on_ota_completed_cb(OTA::ERR::SUCCESS);
  xTimerDelete(xTimer, 0);
}

static esp_err_t ota_handler(httpd_req_t *req) {
  OTA::TYPE ota_type = static_cast<OTA::TYPE>((int)req->user_ctx);
  char buf[2048];
  OTA::ERR ret;
  int recv_len;
  size_t total_received = 0;
  TimerHandle_t delayed_reset_timer = NULL;

  ESP_LOGD(TAG, "Handling route for OTA type %d", (int)ota_type);

  if ((ret = OTA::open(ota_type)) != OTA::ERR::SUCCESS) {
    ESP_LOGE(TAG, "Failed starting OTA: %s", OTA::err_to_str(ret));
    return httpd_resp_send_500(req);
  }
  while (req->content_len - total_received > 0) {
    if ((recv_len = httpd_req_recv(req, buf, 2048)) <= 0) {
      if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
        continue;
      break;
    }
    total_received += recv_len;
    if ((ret = OTA::write((uint8_t *)buf, recv_len)) != OTA::ERR::SUCCESS) {
      ESP_LOGE(TAG, "Failed writing OTA: %s", OTA::err_to_str(ret));
      break;
    }
  }
  if ((ret = OTA::close()) != OTA::ERR::SUCCESS) {
    ESP_LOGE(TAG, "Failed completing OTA: %s", OTA::err_to_str(ret));
    return httpd_resp_send_500(req);
  }
  httpd_resp_sendstr(req, "OK");

  delayed_reset_timer =
      xTimerCreate("delayed_reset", pdMS_TO_TICKS(1000), pdFALSE, req->user_ctx, ota_delayed_reset_timer_cb);
  xTimerStart(delayed_reset_timer, 0);

  return ESP_OK;
}

static esp_err_t static_file_handler(httpd_req_t *req) {
  httpd_static_file *static_file = (httpd_static_file *)req->user_ctx;

  ESP_LOGD(TAG, "Handling route for %s", static_file->path);
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)static_file->start, static_file->end - static_file->start);
}

esp_err_t httpd::register_static_routes(httpd_handle_t server) {
  httpd_static_file *static_file;
  httpd_uri_t uri_static_file = {
      .uri = NULL,
      .method = HTTP_GET,
      .handler = static_file_handler,
      .user_ctx = NULL,
  };

  for (static_file = httpd_static_files; static_file->path; static_file++) {
    ESP_LOGD(TAG, "Registerting route %s", static_file->path);
    uri_static_file.uri = static_file->path;
    uri_static_file.user_ctx = static_file;
    httpd_register_uri_handler(server, &uri_static_file);
  }

  return ESP_OK;
}

esp_err_t httpd::register_ota_routes(httpd_handle_t server) {
  httpd_uri_t uri_ota = {
      .uri = NULL,
      .method = HTTP_POST,
      .handler = ota_handler,
      .user_ctx = NULL,
  };

  uri_ota.uri = "/ota/firmware";
  uri_ota.user_ctx = (void *)OTA::TYPE::FIRMWARE;
  httpd_register_uri_handler(server, &uri_ota);
  uri_ota.uri = "/ota/configuration";
  uri_ota.user_ctx = (void *)OTA::TYPE::CONFIG;
  httpd_register_uri_handler(server, &uri_ota);

  return 0;
}

void httpd::set_on_ota_completed_cb(httpd_on_ota_completed_cb_t cb) {
  using namespace std::placeholders;
  on_ota_completed_cb = std::bind(cb, _1);
}

esp_err_t httpd::init() {
  ESP_LOGI(TAG, "Initializing HTTP server");

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  config.max_uri_handlers = 20;
  config.stack_size = 8192;
  ESP_ERROR_CHECK(httpd_start(&server, &config));

  /* Register URI handlers */
  register_ota_routes(server);
  register_static_routes(server);

  return ESP_OK;
}
} // namespace httpd
