#pragma once

#ifndef OTA_H
#define OTA_H

#include <esp_err.h>
#include <functional>
#include <stddef.h>
#include <stdint.h>

namespace OTA {
enum class TYPE : const uint8_t {
  FIRMWARE,
  CONFIG,
};

enum class ERR : const uint8_t {
  SUCCESS,
  NO_CHANGE,
  IN_PROGRESS,
  FAILED_DOWNLOAD,
  FAILED_BEGIN,
  FAILED_WRITE,
  FAILED_END,
};

using on_completed_cb_t = std::function<void(TYPE type, ERR err)>;

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

ERR download(TYPE type, const char *url, on_completed_cb_t cb);
ERR open(TYPE type);
ERR write(uint8_t *data, size_t len);
ERR close();

static const char *err_to_str(ERR err);

} // namespace OTA
#endif
