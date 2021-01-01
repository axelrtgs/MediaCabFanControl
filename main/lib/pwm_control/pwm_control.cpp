#include "pwm_control.h"

namespace PWMControl {
esp_err_t PWMControl::init() {
  esp_err_t rv = _max31790->init();
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;

  MAX31790::CONFIG config{.watchDogStatus = MAX31790::Watch_Dog_Status::I2C_Occured,
                          .watchDogPeriod = MAX31790::Watch_Dog_Period::Sec_30,
                          .oscillatorSelect = MAX31790::Oscillator_Select::Internal,
                          .busTimeout = MAX31790::Bus_Timeout::Enabled,
                          .reset = MAX31790::Reset::Normal,
                          .runStandby = MAX31790::Run_Standby::Run};
  rv = _max31790->writeConfig(config);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;

  MAX31790::FANOPTIONS fanOptions{.fanFaultQueue = MAX31790::Fan_Fault_Queue::FFQ_6,
                                  .fanFailOptions = MAX31790::Fan_Fail_Options::Duty_100_Unmasked,
                                  .seqStartDelay = MAX31790::Seq_Start_Delay::Sec_4_1};
  rv = _max31790->writeFanOptions(fanOptions);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;

  MAX31790::FANCONFIG fanConfigPWMOutput{.outputMode = MAX31790::Output_Mode::PWM_Output,
                                         .lockedRotorPolarity = MAX31790::Locked_Rotor_Polarity::High,
                                         .tachInputMode = MAX31790::Tach_Input_Mode::Tach_Count,
                                         .tachInputEnable = MAX31790::Tach_Input_Enable::Enabled,
                                         .controlMonitor = MAX31790::Control_Monitor::Control,
                                         .spinUp = MAX31790::Spin_Up::Sec_1,
                                         .mode = MAX31790::Mode::PWM};

  MAX31790::FANCONFIG fanConfigTACHInput{.outputMode = MAX31790::Output_Mode::Tach_Input,
                                         .lockedRotorPolarity = MAX31790::Locked_Rotor_Polarity::High,
                                         .tachInputMode = MAX31790::Tach_Input_Mode::Tach_Count,
                                         .tachInputEnable = MAX31790::Tach_Input_Enable::Enabled,
                                         .controlMonitor = MAX31790::Control_Monitor::Control,
                                         .spinUp = MAX31790::Spin_Up::Sec_1,
                                         .mode = MAX31790::Mode::PWM};

  for (int i = 0; i < NR_CHANNEL; i++) {
    switch (i) {
    case 0:
    case 1:
      rv = _max31790->writeFanConfig(i, fanConfigTACHInput);
      if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;
      break;

    case 2:
    case 3:
    case 4:
    case 5:
      rv = _max31790->writeFanConfig(i, fanConfigPWMOutput);
      if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;
      break;

    default:
      return ESP_ERR_INVALID_ARG;
    }

    rv = _max31790->writeWindow(i, 0);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
  }

  MAX31790::PWMFREQ pwmFreq{.PWM1_3 = MAX31790::PWMFreq::kHz_25, .PWM4_6 = MAX31790::PWMFreq::kHz_25};

  rv = _max31790->writePWMFreq(pwmFreq);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;

  return ESP_OK;
}

esp_err_t PWMControl::getTachRPMByIndex(const uint8_t &index, uint16_t *tachRPM) {
  uint8_t rv = _max31790->readTachRPM(index, tachRPM);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;
  return ESP_OK;
}

esp_err_t PWMControl::getTachRPMComplete(uint16_t *tachRPM) {
  for (int i = 0; i < NUM_FANS; i++) {
    uint16_t rpm;
    uint8_t rv = getTachRPMByIndex(i, &rpm);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
    tachRPM[i] = rpm;
  }
  return ESP_OK;
}

esp_err_t PWMControl::setPWMTargetByIndex(const uint8_t &index, const uint16_t &pwmTarget) {
  uint8_t rv = _max31790->writePWMTarget(index, pwmTarget);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;
  return ESP_OK;
}

esp_err_t PWMControl::setPWMTargetComplete(const uint16_t *pwmTarget) {
  for (int i = 0; i < NUM_PWM; i++) {
    uint8_t rv = setPWMTargetByIndex(PWM_INDEXES[i], pwmTarget[i]);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
  }
  return ESP_OK;
}

esp_err_t PWMControl::getPWMTargetByIndex(const uint8_t &index, uint16_t *pwmTarget) {
  uint8_t rv = _max31790->readPWMTarget(index, pwmTarget);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;
  return ESP_OK;
}

esp_err_t PWMControl::getPWMTargetComplete(uint16_t *pwmTarget) {
  for (int i = 0; i < NUM_PWM; i++) {
    uint16_t pwm;
    uint8_t rv = getPWMTargetByIndex(PWM_INDEXES[i], &pwm);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
    pwmTarget[i] = pwm;
  }
  return ESP_OK;
}

esp_err_t PWMControl::getPWMDutyByIndex(const uint8_t &index, uint16_t *pwmDuty) {
  uint8_t rv = _max31790->readPWMDuty(index, pwmDuty);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;
  return ESP_OK;
}

esp_err_t PWMControl::getPWMDutyComplete(uint16_t *pwmDuty) {
  for (int i = 0; i < NUM_PWM; i++) {
    uint16_t pwm;
    uint8_t rv = getPWMDutyByIndex(PWM_INDEXES[i], &pwm);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
    pwmDuty[i] = pwm;
  }
  return ESP_OK;
}

esp_err_t PWMControl::getPWMDutyPercentByIndex(const uint8_t &index, uint8_t *pwmDutyPercent) {
  uint16_t pwm;
  uint8_t rv = _max31790->readPWMDuty(index, &pwm);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;

  *pwmDutyPercent = pwm / PWM_MAX;
  return ESP_OK;
}

esp_err_t PWMControl::getPWMDutyPercentComplete(uint8_t *pwmDutyPercent) {
  for (int i = 0; i < NUM_PWM; i++) {
    uint8_t pwmPercent;
    uint8_t rv = getPWMDutyPercentByIndex(PWM_INDEXES[i], &pwmPercent);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
    pwmDutyPercent[i] = pwmPercent;
  }
  return ESP_OK;
}
} // namespace PWMControl
