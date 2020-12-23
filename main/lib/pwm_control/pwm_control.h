#pragma once

#ifndef PWMControl_h
#define PWMControl_h

#include "esp_err.h"
#include "max31790.h"

namespace {
const uint8_t NUM_FANS = 8;
const uint8_t NUM_PWM = 4;
}  // namespace

class PWMControl {
 public:
  PWMControl(I2C_t *i2c, const uint8_t &deviceAddress) {
    _max31790 = new MAX31790(i2c, deviceAddress);
  }

  esp_err_t init();

  esp_err_t getTachRPMByIndex(const uint8_t &index, uint16_t *tachRPM);

  esp_err_t getTachRPMComplete(uint16_t *tachRPM);

  esp_err_t setPWMTargetByIndex(const uint8_t &index,
                                const uint16_t &pwmTarget);

  esp_err_t setPWMTargetComplete(const uint16_t *pwmTarget);

  esp_err_t getPWMTargetByIndex(const uint8_t &index, uint16_t *pwmTarget);

  esp_err_t getPWMTargetComplete(uint16_t *pwmTarget);

  esp_err_t getPWMDutyByIndex(const uint8_t &index, uint16_t *pwmDuty);

  esp_err_t getPWMDutyComplete(uint16_t *pwmDuty);

  esp_err_t getPWMDutyPercentByIndex(const uint8_t &index,
                                     uint8_t *pwmDutyPercent);

  esp_err_t getPWMDutyPercentComplete(uint8_t *pwmDutyPercent);

 private:
  MAX31790 *_max31790;
};
#endif
