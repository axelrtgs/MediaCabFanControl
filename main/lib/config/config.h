#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include <esp_err.h>
#include <stddef.h>
#include <stdint.h>

namespace Config {
/* Types */
typedef struct config_update_handle_t config_update_handle_t;
typedef enum config_network_type_t {
  NETWORK_TYPE_WIFI,
  NETWORK_TYPE_ETH,
} config_network_type_t;

/* MQTT Configuration*/
const char *config_mqtt_host_get(void);
uint16_t config_mqtt_port_get(void);
uint8_t config_mqtt_ssl_get(void);
const char *config_mqtt_server_cert_get(void);
const char *config_mqtt_client_cert_get(void);
const char *config_mqtt_client_key_get(void);
const char *config_mqtt_client_id_get(void);
const char *config_mqtt_username_get(void);
const char *config_mqtt_password_get(void);
uint8_t config_mqtt_qos_get(void);
uint8_t config_mqtt_retained_get(void);
const char *config_mqtt_prefix_get(void);
const char *config_mqtt_get_suffix_get(void);
const char *config_mqtt_set_suffix_get(void);

/* Remote Logging Configuration */
const char *config_log_host_get(void);
uint16_t config_log_port_get(void);

/* Configuration Update */
esp_err_t config_update_begin(config_update_handle_t **handle);
esp_err_t config_update_write(config_update_handle_t *handle, uint8_t *data, size_t len);
esp_err_t config_update_end(config_update_handle_t *handle);

const char *config_version_get(void);
esp_err_t config_initialize(void);
} // namespace Config
#endif
