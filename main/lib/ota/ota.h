#pragma once

#ifndef OTA_H
#define OTA_H

#include <esp_err.h>
#include <functional>
#include <stddef.h>
#include <stdint.h>

#define ESP_ERR_OTA_DOWNLOAD_FAILED (ESP_ERR_OTA_BASE + 0x07)
#define ESP_ERR_OTA_NO_CHANGE (ESP_ERR_OTA_BASE + 0x07)
#define ESP_ERR_OTA_IN_PROGRESS (ESP_ERR_OTA_BASE + 0x07)
#define ESP_ERR_OTA_BEGIN_FAILED (ESP_ERR_OTA_BASE + 0x07)
#define ESP_ERR_OTA_WRITE_FAILED (ESP_ERR_OTA_BASE + 0x07)
#define ESP_ERR_OTA_END_FAILED (ESP_ERR_OTA_BASE + 0x07)

namespace {
  uint8_t OTA_URL_SIZE = 256;
  uint16_t OTA_RECV_TIMEOUT = 5000;
  uint16_t OTA_BUFFER_SIZE = 2048;
  bool USE_GLOBAL_CA_STORE = true;
  bool SKIP_CERT_COMMON_NAME_CHECK = true;
}
namespace OTA {
enum class TYPE : const uint8_t {
  FIRMWARE,
  CONFIG,
};

using on_completed_cb_t = std::function<void(esp_err_t err)>;

typedef struct {
  TYPE type;
  esp_err_t (*begin)(void **handle);
  esp_err_t (*write)(void *handle, uint8_t *data, size_t len);
  esp_err_t (*end)(void *handle);
  const char *(*version_get)(void);
} ops_t;

typedef struct {
  char *url;
  on_completed_cb_t on_completed_cb;
} download_ctx;

struct {
  int in_progress;
  ops_t *ops;
  size_t bytes_written;
  void *handle;
} ctx;

esp_err_t download(TYPE type, const char *url, on_completed_cb_t cb);
esp_err_t open(TYPE type);
esp_err_t write(uint8_t *data, size_t len);
esp_err_t close();
} // namespace OTA
#endif
