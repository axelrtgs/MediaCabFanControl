#pragma once

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"

#include "esp_bt.h"
#include "esp_blufi_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "lwip/inet.h"
#include <driver/gpio.h>

#include "mbedtls/aes.h"
#include "mbedtls/dhm.h"
#include "mbedtls/md5.h"
#include "esp32/rom/crc.h"

#define BLUFI_WIRELESS_TAG "BLUFI_WIRELESS"
#define BLUFI_DEVICE_NAME  "BLUFI_DEVICE"

#define BLUFI_INFO(fmt, ...)   ESP_LOGI(BLUFI_WIRELESS_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_WARN(fmt, ...)   ESP_LOGW(BLUFI_WIRELESS_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_ERROR(fmt, ...)  ESP_LOGE(BLUFI_WIRELESS_TAG, fmt, ##__VA_ARGS__)

// If defined and a button connected to the pin specified is held down will delete Wifi config from NVS
// Set to boot button on dev board, need to test if changing
#define BLUFI_OVERRIDE_GPIO 0

#ifdef __cplusplus
extern "C" {
#endif

extern EventGroupHandle_t wifi_event_group;
extern const int CONNECTED_BIT;

esp_err_t  blufi_init();

void blufi_dh_negotiate_data_handler(uint8_t *data, int len, uint8_t **output_data, int *output_len, bool *need_free);
int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);
int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);
uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len);

int blufi_security_init(void);
void blufi_security_deinit(void);

#ifdef __cplusplus
}
#endif
