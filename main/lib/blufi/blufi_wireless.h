#pragma once

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "esp_blufi_api.h"
#include "esp_gap_ble_api.h"

#include "blufi_security.h"
#include "blufi.h"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
extern EventGroupHandle_t wifi_event_group;

/* store the station info for send back to phone */
extern bool gl_sta_connected;

extern esp_blufi_callbacks_t blufi_callbacks;

const int WIFI_GOT_IP_BIT = BIT0;
const int WIFI_GOT_IP6_BIT = BIT1;

esp_err_t event_handler(void *ctx, system_event_t *event);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);

