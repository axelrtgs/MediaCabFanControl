#include "blufi_security.h"

struct blufi_security *blufiSecurity;

void blufi_dh_negotiate_data_handler(uint8_t *data, int len, uint8_t **output_data, int *output_len, bool *need_free)
{
    int ret;
    uint8_t type = data[0];

    if (blufiSecurity == NULL) {
        BLUFI_ERROR("%s BLUFI Security is not initialized\n", __func__);
        btc_blufi_report_error(ESP_BLUFI_INIT_SECURITY_ERROR);
        return;
    }

    switch (type) {
        case SEC_TYPE_DH_PARAM_LEN:
            blufiSecurity->dh_param_len = ((data[1]<<8)|data[2]);
            if (blufiSecurity->dh_param) {
                free(blufiSecurity->dh_param);
                blufiSecurity->dh_param = NULL;
            }
            blufiSecurity->dh_param = (uint8_t *)malloc(blufiSecurity->dh_param_len);
            if (blufiSecurity->dh_param == NULL) {
                btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                BLUFI_ERROR("%s, malloc failed\n", __func__);
                return;
            }
            break;
        case SEC_TYPE_DH_PARAM_DATA: {
            if (blufiSecurity->dh_param == NULL) {
                BLUFI_ERROR("%s, blufiSecurity->dh_param == NULL\n", __func__);
                btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
                return;
            }
            uint8_t* param = blufiSecurity->dh_param;
            memcpy(blufiSecurity->dh_param, &data[1], blufiSecurity->dh_param_len);
            ret = mbedtls_dhm_read_params(&blufiSecurity->dhm, &param, &param[blufiSecurity->dh_param_len]);
            if (ret) {
                BLUFI_ERROR("%s read param failed %d\n", __func__, ret);
                btc_blufi_report_error(ESP_BLUFI_READ_PARAM_ERROR);
                return;
            }
            free(blufiSecurity->dh_param);
            blufiSecurity->dh_param = NULL;
            ret = mbedtls_dhm_make_public(&blufiSecurity->dhm,
                                          (int) mbedtls_mpi_size(&blufiSecurity->dhm.P),
                                          blufiSecurity->self_public_key,
                                          blufiSecurity->dhm.len,
                                          myrand,
                                          NULL);
            if (ret) {
                BLUFI_ERROR("%s make public failed %d\n", __func__, ret);
                btc_blufi_report_error(ESP_BLUFI_MAKE_PUBLIC_ERROR);
                return;
            }

            mbedtls_dhm_calc_secret(&blufiSecurity->dhm, blufiSecurity->share_key, SHARE_KEY_BIT_LEN, &blufiSecurity->share_len, myrand, NULL);

            mbedtls_md5(blufiSecurity->share_key, blufiSecurity->share_len, blufiSecurity->psk);

            mbedtls_aes_setkey_enc(&blufiSecurity->aes, blufiSecurity->psk, 128);
            mbedtls_aes_setkey_dec(&blufiSecurity->aes, blufiSecurity->psk, 128);

            /* alloc output data */
            *output_data = &blufiSecurity->self_public_key[0];
            *output_len = blufiSecurity->dhm.len;
            *need_free = false;
            break;
        }
        case SEC_TYPE_DH_P:
            break;
        case SEC_TYPE_DH_G:
            break;
        case SEC_TYPE_DH_PUBLIC:
            break;
    }
}

int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len)
{
    int ret;
    size_t iv_offset = 0;
    uint8_t iv0[16];

    memcpy(iv0, blufiSecurity->iv, sizeof(blufiSecurity->iv));
    iv0[0] = iv8;   /* set iv8 as the iv0[0] */

    ret = mbedtls_aes_crypt_cfb128(&blufiSecurity->aes, MBEDTLS_AES_ENCRYPT, crypt_len, &iv_offset, iv0, crypt_data, crypt_data);
    if (ret) {
        return -1;
    }

    return crypt_len;
}

int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len)
{
    int ret;
    size_t iv_offset = 0;
    uint8_t iv0[16];

    memcpy(iv0, blufiSecurity->iv, sizeof(blufiSecurity->iv));
    iv0[0] = iv8;   /* set iv8 as the iv0[0] */

    ret = mbedtls_aes_crypt_cfb128(&blufiSecurity->aes, MBEDTLS_AES_DECRYPT, crypt_len, &iv_offset, iv0, crypt_data, crypt_data);
    if (ret) {
        return -1;
    }

    return crypt_len;
}

uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len)
{
    /* This iv8 ignore, not used */
    return crc16_be(0, data, len);
}

esp_err_t blufi_security_init(void)
{
    blufiSecurity = (struct blufi_security *)malloc(sizeof(struct blufi_security));
    if (blufiSecurity == NULL) {
        return ESP_FAIL;
    }

    memset(blufiSecurity, 0x0, sizeof(struct blufi_security));

    mbedtls_dhm_init(&blufiSecurity->dhm);
    mbedtls_aes_init(&blufiSecurity->aes);

    memset(blufiSecurity->iv, 0x0, 16);
    return ESP_OK;
}

void blufi_security_deinit(void)
{
    if (blufiSecurity == NULL) {
        return;
    }
    if (blufiSecurity->dh_param){
        free(blufiSecurity->dh_param);
        blufiSecurity->dh_param = NULL;
    }
    mbedtls_dhm_free(&blufiSecurity->dhm);
    mbedtls_aes_free(&blufiSecurity->aes);

    memset(blufiSecurity, 0x0, sizeof(struct blufi_security));

    free(blufiSecurity);
    blufiSecurity =  NULL;
}
