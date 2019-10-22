#pragma once


#include "esp_log.h"

#define BLUFI_WIRELESS_TAG "BLUFI"
#define BLUFI_INFO(fmt, ...)   ESP_LOGI(BLUFI_WIRELESS_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_WARN(fmt, ...)   ESP_LOGW(BLUFI_WIRELESS_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_ERROR(fmt, ...)  ESP_LOGE(BLUFI_WIRELESS_TAG, fmt, ##__VA_ARGS__)