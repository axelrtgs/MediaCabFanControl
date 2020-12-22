#pragma once

#include <driver/gpio.h>
#include <cmath>

#include <esp_log.h>

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

namespace
{
  const uint8_t MAX_DEVICES = 8;
  const DS18B20_RESOLUTION TEMPERATURE_RESOLUTION = DS18B20_RESOLUTION_12_BIT;
}

namespace temperature
{
  static const char *TAG = "temperature";

  struct temperature_reading
  {
    float value = 0;
    DS18B20_ERROR error = DS18B20_OK;
  };

  class temperature
  {
  public:
    temperature() = default;

    void init(uint8_t gpio_pin, uint8_t instance_id)
    {
      // Stable readings require a brief period before communication
      vTaskDelay(2000.0 / portTICK_PERIOD_MS);

      gpio_num_t gpio_num = static_cast<gpio_num_t>(gpio_pin);
      ESP_LOGD(TAG, "Find temperature devices on OneWire bus gpio #: %d:", gpio_num);

      rmt_channel_t tx_channel = static_cast<rmt_channel_t>(RMT_CHANNEL_0 + instance_id * 2);
      rmt_channel_t rx_channel = static_cast<rmt_channel_t>(RMT_CHANNEL_1 + instance_id * 2);

      owb = owb_rmt_initialize(&rmt_driver_info, gpio_num, tx_channel, rx_channel);
      owb_use_crc(owb, true);

      std::vector<OneWireBus_ROMCode> device_rom_codes;
      OneWireBus_SearchState search_state;
      bool device_found = false;
      owb_search_first(owb, &search_state, &device_found);
      while (device_found)
      {
        char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));

        ESP_LOGD(TAG, "Found DS18B20 device ROM code: %s", rom_code_s);
        device_rom_codes.emplace_back(search_state.rom_code);
        owb_search_next(owb, &search_state, &device_found);
      }

      int num_devices = device_rom_codes.size();
      ESP_LOGD(TAG, "Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");

      for(const auto& device_rom_code: device_rom_codes)
      {
        DS18B20_Info * ds18b20_info = ds18b20_malloc();
        devices.emplace_back(ds18b20_info);

        if (device_rom_codes.size() == 1)
        {
          ESP_LOGD(TAG, "Single device optimisations enabled");
          ds18b20_init_solo(ds18b20_info, owb);
        }
        else
          ds18b20_init(ds18b20_info, owb, device_rom_code);

        ds18b20_use_crc(ds18b20_info, true);
        ds18b20_set_resolution(ds18b20_info, TEMPERATURE_RESOLUTION);
      }
    }

    float average_temperature()
    {
      std::vector<temperature_reading> temperature_vector = get_temps();
      float sum = 0.0;
      for(const auto& temperature: temperature_vector)
        sum += temperature.value;

      return sum / temperature_vector.size();
    }

    std::vector<temperature_reading> get_temps()
    {
      std::vector<temperature_reading> readings;

      if (devices.size() > 0)
      {
        ds18b20_convert_all(owb);
        ds18b20_wait_for_conversion(devices[0]);

        for(const auto& device: devices)
        {
          temperature_reading reading;
          reading.error = ds18b20_read_temp(device, &reading.value);
          readings.emplace_back(reading);
        }

        for(const auto& reading: readings)
        {
          if (reading.error != DS18B20_OK)
          {
            ESP_LOGE(TAG, "Couldn't read data from temperature sensor");
            std::vector<temperature_reading> empty;
            return empty;
          }
          else
            ESP_LOGD(TAG, "Temperature sensor reading (degrees C): %.1f", reading.value);
        }
      }
      else
        ESP_LOGI(TAG, "No DS18B20 temperature devices detected!");

      return readings;
    }
  private:
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    std::vector<DS18B20_Info*> devices;
  };
}
