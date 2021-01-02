#include "ota.h"
#include "config.h"
#include <esp_err.h>
#include <esp_http_client.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stddef.h>
#include <string.h>

namespace OTA {
static const char *TAG = "OTA";

/* Config OTA Wrappers */
static esp_err_t config_begin(void **handle) {
  return Config::config_update_begin((Config::config_update_handle_t **)handle);
}

static esp_err_t config_write(void *handle, uint8_t *data, size_t len) {
  return Config::config_update_write((Config::config_update_handle_t *)handle, data, len);
}

static esp_err_t config_end(void *handle) {
  return Config::config_update_end((Config::config_update_handle_t *)handle);
}

static const char *config_version_get(void) { return Config::config_version_get(); }

static ops_t config_ops = {
    .type = TYPE::CONFIG,
    .begin = config_begin,
    .write = config_write,
    .end = config_end,
    .version_get = config_version_get,
};

/* Firmware OTA Wrappers */
static esp_err_t firmware_begin(void **handle) {
  const esp_partition_t *configured = esp_ota_get_boot_partition();
  const esp_partition_t *running = esp_ota_get_running_partition();
  const esp_partition_t *update = NULL;
  esp_ota_handle_t update_handle = 0;
  esp_err_t err;

  if (configured != running) {
    ESP_LOGW(TAG, "Configured OTA boot partition is different than running partition");
  }

  if (!(update = esp_ota_get_next_update_partition(NULL))) {
    ESP_LOGE(TAG, "Failed getting update partition");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Running partition type 0x%0x subtype 0x%0x (offset 0x%08x)", running->type, running->subtype,
           running->address);
  ESP_LOGI(TAG, "Writing partition type 0x%0x subtype 0x%0x (offset 0x%08x)", update->type, update->subtype,
           update->address);

  err = esp_ota_begin(update, OTA_SIZE_UNKNOWN, &update_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed beginning OTA: 0x%x", err);
    return ESP_FAIL;
  }

  *handle = (void *)update_handle;

  return ESP_OK;
}

static esp_err_t firmware_write(void *handle, uint8_t *data, size_t len) {
  esp_ota_handle_t update_handle = (esp_ota_handle_t)handle;
  esp_err_t err = esp_ota_write(update_handle, (const void *)data, len);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed writing OTA: 0x%x", err);
    return ESP_FAIL;
  }
  return ESP_OK;
}

static esp_err_t firmware_end(void *handle) {
  esp_ota_handle_t update_handle = (esp_ota_handle_t)handle;
  const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
  esp_err_t err = esp_ota_end(update_handle);

  if (!update || err != ESP_OK) {
    ESP_LOGE(TAG, "Failed ending OTA: 0x%x", err);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Setting boot partition type 0x%0x subtype 0x%0x (offset 0x%08x)", update->type, update->subtype,
           update->address);

  err = esp_ota_set_boot_partition(update);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed setting boot partition: 0x%x", err);
    return ESP_FAIL;
  }

  return ESP_OK;
}

static const char *firmware_version_get(void) { return BLE2MQTT_VER; }

static ops_t firmware_ops = {
    .type = TYPE::FIRMWARE,
    .begin = firmware_begin,
    .write = firmware_write,
    .end = firmware_end,
    .version_get = firmware_version_get,
};

static int http_event_cb(esp_http_client_event_t *event) {
  if (event->event_id != HTTP_EVENT_ON_DATA)
    return ESP_OK;

  if (write((uint8_t *)event->data, event->data_len) != ERR::SUCCESS)
    return ESP_FAIL;

  return ESP_OK;
}

static void task(void *pvParameter) {
  download_ctx *dl_ctx = (download_ctx *)pvParameter;
  esp_http_client_handle_t handle;
  char header[128];
  int http_status = -1;
  ERR err;
  esp_http_client_config_t config = {
      .url = dl_ctx->url,
      .method = HTTP_METHOD_GET,
      .event_handler = http_event_cb,
      .buffer_size = 2048,
  };

  ESP_LOGI(TAG, "Starting OTA from %s", dl_ctx->url);
  handle = esp_http_client_init(&config);

  /* Set HTTP headers */
  sprintf(header, "BLE2MQTT/%s", BLE2MQTT_VER);
  esp_http_client_set_header(handle, "User-Agent", header);
  sprintf(header, "\"%s\"", ctx.ops->version_get());
  esp_http_client_set_header(handle, "If-None-Match", header);

  /* Start HTTP request */
  if (esp_http_client_perform(handle) == ESP_OK)
    http_status = esp_http_client_get_status_code(handle);

  ESP_LOGI(TAG, "HTTP request response: %d, read %d (%d) bytes", http_status,
           esp_http_client_get_content_length(handle), ctx.bytes_written);

  err = close();
  if (http_status != 200 && http_status != 304)
    err = ERR::FAILED_DOWNLOAD;

  if (dl_ctx->on_completed_cb)
    dl_ctx->on_completed_cb(ctx.ops->type, err);

  free(dl_ctx->url);
  free(dl_ctx);
  esp_http_client_cleanup(handle);
  vTaskDelete(NULL);
}

ERR download(TYPE type, const char *url, on_completed_cb_t cb) {
  ERR ret;
  download_ctx *dl_ctx;

  if ((ret = open(type)) != ERR::SUCCESS)
    return ret;

  dl_ctx = (download_ctx *)malloc(sizeof(*dl_ctx));
  dl_ctx->url = strdup(url);
  dl_ctx->on_completed_cb = cb;

  xTaskCreatePinnedToCore(task, "task", 8192, dl_ctx, 5, NULL, 1);

  return ERR::SUCCESS;
}

ERR open(TYPE type) {
  if (ctx.in_progress)
    return ERR::IN_PROGRESS;

  ctx.ops = type == TYPE::FIRMWARE ? &firmware_ops : &config_ops;

  ctx.in_progress = 1;
  ctx.bytes_written = 0;

  return ERR::SUCCESS;
}

ERR write(uint8_t *data, size_t len) {
  if (!ctx.in_progress)
    return ERR::FAILED_WRITE;

  if (!ctx.bytes_written) {
    if (ctx.ops->begin(&ctx.handle))
      return ERR::FAILED_BEGIN;
  }

  if (ctx.ops->write(ctx.handle, data, len)) {
    ESP_LOGE(TAG, "Failed writing data");
    return ERR::FAILED_WRITE;
  }
  ctx.bytes_written += len;
  ESP_LOGI(TAG, "Wrote %d bytes (total: %d)", len, ctx.bytes_written);

  return ERR::SUCCESS;
}

ERR close() {
  if (!ctx.in_progress)
    return ERR::FAILED_END;

  ctx.in_progress = 0;

  if (!ctx.bytes_written)
    return ERR::NO_CHANGE;
  return ctx.ops->end(ctx.handle) ? ERR::FAILED_END : ERR::SUCCESS;
}

const char *err_to_str(ERR err) {
  switch (err) {
  case ERR::SUCCESS:
    return "Success";
  case ERR::NO_CHANGE:
    return "No change";
  case ERR::IN_PROGRESS:
    return "In progress";
  case ERR::FAILED_DOWNLOAD:
    return "Failed downloading file";
  case ERR::FAILED_BEGIN:
    return "Failed initializing OTA process";
  case ERR::FAILED_WRITE:
    return "Failed writing data";
  case ERR::FAILED_END:
    return "Failed finalizing OTA process";
  }

  return "Invalid OTA error";
}
} // namespace OTA
