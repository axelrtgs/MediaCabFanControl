#pragma once

#include <dispatcher.h>
#include <logger.h>
#include "esp_event.h"

#include "freertos/FreeRTOS.h"

#include "esp_blufi_api.h"

#include "mbedtls/aes.h"
#include "mbedtls/dhm.h"
#include "mbedtls/md5.h"
#include "esp_crc.h"

#define BLUFI_DEVICE_NAME "BLUFI_DEVICE"

namespace
{
    /*
        The SEC_TYPE_xxx is for self-defined packet data type in the procedure of "BLUFI negotiate key"
        If user use other negotiation procedure to exchange(or generate) key, should redefine the type by yourself.
    */
    const uint8_t SEC_TYPE_DH_PARAM_LEN = 0x00;
    const uint8_t SEC_TYPE_DH_PARAM_DATA = 0x01;
    const uint8_t SEC_TYPE_DH_P = 0x02;
    const uint8_t SEC_TYPE_DH_G = 0x03;
    const uint8_t SEC_TYPE_DH_PUBLIC = 0x04;
    const uint8_t DH_SELF_PUB_KEY_LEN = 128;
    const uint16_t DH_SELF_PUB_KEY_BIT_LEN = DH_SELF_PUB_KEY_LEN * 8;
    const uint8_t SHARE_KEY_LEN = 128;
    const uint16_t SHARE_KEY_BIT_LEN = SHARE_KEY_LEN * 8;
    const uint8_t PSK_LEN = 16;

    const int WIFI_GOT_IP_BIT = BIT0;
    const int WIFI_GOT_IP6_BIT = BIT1;
}
namespace blufi
{
    struct blufi_security
    {
        uint8_t self_public_key[DH_SELF_PUB_KEY_LEN];
        uint8_t share_key[SHARE_KEY_LEN];
        size_t share_len;
        uint8_t psk[PSK_LEN];
        uint8_t* dh_param;
        int dh_param_len;
        uint8_t iv[16];
        mbedtls_dhm_context dhm;
        mbedtls_aes_context aes;
    };

    extern "C" void btc_blufi_report_error(esp_blufi_error_state_t state);

    inline int myrand(void* rng_state, unsigned char* output, size_t len)
    {
        esp_fill_random(output, len);
        return 0;
    }

    enum class WifiAssociationState: int
    {
        CONNECTED = 0,
        CONNECTED6 = 1,
        DISCONNECTED = 2,
        CONNECTING = 3
    };

    struct WifiConnectionState
    {
        WifiAssociationState wifiState = WifiAssociationState::DISCONNECTED;
        std::string ipAddr = "N/A";
    };

    using wificallback_t = std::function<void(blufi::WifiAssociationState)>;
    using WifiConnectionStateCB = std::function<void(blufi::WifiConnectionState)>;
    class blufi
    {
    public:
        blufi() = default;
        void init();
        void registerCallback(WifiConnectionStateCB callback);

        std::string getIpAddrStr() const;
        WifiConnectionState getWifiState();


    private:
        void wifiStateChanged(WifiAssociationState state);

        Dispatcher<WifiConnectionState> mWifiStateNotifier;
        wificallback_t mWifiCallback; // Need to store so doesnt go out of scope in C land
    };
}
