#pragma once

#include <string.h>

#include "freertos/FreeRTOS.h"

#include "esp_blufi_api.h"

#include "mbedtls/aes.h"
#include "mbedtls/dhm.h"
#include "mbedtls/md5.h"
#include "esp32/rom/crc.h"

#include "blufi.h"

/*
    The SEC_TYPE_xxx is for self-defined packet data type in the procedure of "BLUFI negotiate key"
    If user use other negotiation procedure to exchange(or generate) key, should redefine the type by yourself.
*/
const uint8_t SEC_TYPE_DH_PARAM_LEN     = 0x00;
const uint8_t SEC_TYPE_DH_PARAM_DATA    = 0x01;
const uint8_t SEC_TYPE_DH_P             = 0x02;
const uint8_t SEC_TYPE_DH_G             = 0x03;
const uint8_t SEC_TYPE_DH_PUBLIC        = 0x04;
const uint8_t DH_SELF_PUB_KEY_LEN       = 128;
const uint16_t DH_SELF_PUB_KEY_BIT_LEN  = DH_SELF_PUB_KEY_LEN * 8;
const uint8_t SHARE_KEY_LEN             = 128;
const uint16_t SHARE_KEY_BIT_LEN        = SHARE_KEY_LEN * 8;
const uint8_t PSK_LEN                   = 16;

struct blufi_security {
    uint8_t  self_public_key[DH_SELF_PUB_KEY_LEN];
    uint8_t  share_key[SHARE_KEY_LEN];
    size_t   share_len;
    uint8_t  psk[PSK_LEN];
    uint8_t  *dh_param;
    int      dh_param_len;
    uint8_t  iv[16];
    mbedtls_dhm_context dhm;
    mbedtls_aes_context aes;
};

extern "C" void btc_blufi_report_error(esp_blufi_error_state_t state);

inline int myrand( void *rng_state, unsigned char *output, size_t len )
{
    esp_fill_random(output, len);
    return 0;
}

void blufi_dh_negotiate_data_handler(uint8_t *data, int len, uint8_t **output_data, int *output_len, bool *need_free);
int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);
int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);
uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len);

int blufi_security_init(void);
void blufi_security_deinit(void);
