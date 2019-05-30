#pragma once

#include <string.h>
#include "esp_log.h"

#include "esp_event_loop.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include <driver/gpio.h>

#include "blufi_security.h"
#include "blufi_wireless.h"

#define BLUFI_WIRELESS_TAG "BLUFI"
#define BLUFI_DEVICE_NAME "BLUFI_DEVICE"

#define BLUFI_INFO(fmt, ...)   ESP_LOGI(BLUFI_WIRELESS_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_WARN(fmt, ...)   ESP_LOGW(BLUFI_WIRELESS_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_ERROR(fmt, ...)  ESP_LOGE(BLUFI_WIRELESS_TAG, fmt, ##__VA_ARGS__)

// If set to a value other than `GPIO_NUM_NC` when toggled will bypass wifi and start BT config
// Set to boot button on dev board, need to test if changing
const gpio_num_t BLUFI_OVERRIDE_GPIO = GPIO_NUM_0;

esp_err_t  blufi_init();
