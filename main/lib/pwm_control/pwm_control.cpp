#include "pwm_control.h"

esp_err_t FanControl::init()
{
  esp_err_t rv = _max31790->init();
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;

  MAX31790::CONFIG config
  {
    .watchDogStatus = MAX31790::Watch_Dog_Status::I2C_Occured,
    .watchDogPeriod = MAX31790::Watch_Dog_Period::Sec_30,
    .oscillatorSelect = MAX31790::Oscillator_Select::Internal,
    .busTimeout = MAX31790::Bus_Timeout::Enabled,
    .reset = MAX31790::Reset::Normal,
    .runStandby = MAX31790::Run_Standby::Run
  };
  rv = _max31790->writeConfig(config);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;

  MAX31790::FANOPTIONS fanOptions
  {
    .fanFaultQueue = MAX31790::Fan_Fault_Queue::FFQ_6,
    .fanFailOptions = MAX31790::Fan_Fail_Options::Duty_100_Unmasked,
    .seqStartDelay = MAX31790::Seq_Start_Delay::Sec_4_1
  };
  rv = _max31790->writeFanOptions(fanOptions);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;

  MAX31790::FANCONFIG fanConfigPWMOutput
  {
    .outputMode = MAX31790::Output_Mode::PWM_Output,
    .lockedRotorPolarity = MAX31790::Locked_Rotor_Polarity::High,
    .tachInputMode = MAX31790::Tach_Input_Mode::Tach_Count,
    .tachInputEnable = MAX31790::Tach_Input_Enable::Enabled,
    .controlMonitor = MAX31790::Control_Monitor::Control,
    .spinUp = MAX31790::Spin_Up::Sec_1,
    .mode = MAX31790::Mode::PWM
  };

  MAX31790::FANCONFIG fanConfigTACHInput
  {
    .outputMode = MAX31790::Output_Mode::Tach_Input,
    .lockedRotorPolarity = MAX31790::Locked_Rotor_Polarity::High,
    .tachInputMode = MAX31790::Tach_Input_Mode::Tach_Count,
    .tachInputEnable = MAX31790::Tach_Input_Enable::Enabled,
    .controlMonitor = MAX31790::Control_Monitor::Control,
    .spinUp = MAX31790::Spin_Up::Sec_1,
    .mode = MAX31790::Mode::PWM
  };

  for (int i = 1; i <= NR_CHANNEL; i++) {
    switch (i) {
      case 1 :
      case 2 :
      case 3 :
        rv = _max31790->writeFanConfig(i, fanConfigTACHInput);
        if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
          return rv;
        break;

      case 4 :
      case 5 :
      case 6 :
        rv = _max31790->writeFanConfig(i, fanConfigPWMOutput);
        if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
          return rv;
        break;

      default :
        return ESP_ERR_INVALID_ARG;
    }

    rv = _max31790->writeWindow(i, 0);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
  }

  return ESP_OK;
}

esp_err_t FanControl::getTachRPMByIndex(const uint8_t& index, uint16_t* tachRPM)
{
  uint8_t rv = _max31790->readTachRPM(index, tachRPM);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;
  return ESP_OK;
}

esp_err_t FanControl::getTachRPMComplete(uint16_t* tachRPM)
{
  for (int i = 0; i < NUM_FANS; i++) {
    uint16_t rpm;
    uint8_t rv = getTachRPMByIndex(i, &rpm);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
    tachRPM[i] = rpm;
  }
  return ESP_OK;
}

esp_err_t FanControl::setPWMTargetByIndex(const uint8_t& index, const uint16_t& pwmTarget)
{
  uint8_t rv = _max31790->writePWMTarget(index, pwmTarget);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;
  return ESP_OK;
}

esp_err_t FanControl::setPWMTargetComplete(const uint16_t* pwmTarget)
{
  for (int i = 0; i < NUM_PWM; i++) {
    uint8_t rv = setPWMTargetByIndex(i, pwmTarget[i]);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
  }
  return ESP_OK;
}

esp_err_t FanControl::getPWMTargetByIndex(const uint8_t& index, uint16_t* pwmTarget)
{
  uint8_t rv = _max31790->readPWMTarget(index, pwmTarget);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;
  return ESP_OK;
}

esp_err_t FanControl::getPWMTargetComplete(uint16_t* pwmTarget)
{
  for (int i = 0; i < NUM_PWM; i++) {
    uint16_t pwm;
    uint8_t rv = getPWMTargetByIndex(i, &pwm);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
    pwmTarget[i] = pwm;
  }
  return ESP_OK;
}

esp_err_t FanControl::getPWMDutyByIndex(const uint8_t& index, uint16_t* pwmDuty)
{
  uint8_t rv = _max31790->readPWMDuty(index, pwmDuty);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;
  return ESP_OK;
}

esp_err_t FanControl::getPWMDutyComplete(uint16_t* pwmDuty)
{
  for (int i = 0; i < NUM_PWM; i++) {
    uint16_t pwm;
    uint8_t rv = getPWMDutyByIndex(i, &pwm);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
    pwmDuty[i] = pwm;
  }
  return ESP_OK;
}

esp_err_t FanControl::getPWMDutyPercentByIndex(const uint8_t& index, uint8_t* pwmDutyPercent)
{
  uint16_t pwm;
  uint8_t rv = _max31790->readPWMDuty(index, &pwm);
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
    return rv;

  *pwmDutyPercent = pwm / PWM_MAX;
  return ESP_OK;
}

esp_err_t FanControl::getPWMDutyPercentComplete(uint8_t* pwmDutyPercent)
{
  for (int i = 0; i < NUM_PWM; i++) {
    uint8_t pwmPercent;
    uint8_t rv = getPWMDutyPercentByIndex(i, &pwmPercent);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
      return rv;
    pwmDutyPercent[i] = pwmPercent;
  }
  return ESP_OK;
}
