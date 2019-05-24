#pragma once

#include <DallasTemperature.h>
#include "FanControl.h"
#include "PIDEnhanced.h"
#include "I2C.h"
#include "Temperature.h"
#include "Utilities.h"

#ifndef Zone_h
#define Zone_h

namespace
{
const uint8_t TEMPERATURE_SENSOR_PIN = 2;
const bool  I2CFAST = true;
const uint8_t MAX31790_ADDR = 47;

PIDEnhanced::PID_TUNING conservative_tune
{
  .Kp = -10.0,
  .Ki = 0.0,
  .Kd = 0.0
};

PIDEnhanced::PID_TUNING aggressive_tune
{
  .Kp = -10.0,
  .Ki = 0.0,
  .Kd = 0.0
};

DeviceAddress temperatureSensorAddress[4] = {
  { 0x28, 0x97, 0x23, 0xE5, 0x8, 0x0, 0x0, 0x71 },
  { 0x28, 0x27, 0x1B, 0xE5, 0x8, 0x0, 0x0, 0xCC },
  { 0x28, 0x27, 0x1B, 0xE5, 0x8, 0x0, 0x0, 0xCC },
  { 0x28, 0x27, 0x1B, 0xE5, 0x8, 0x0, 0x0, 0xCC }
};
}

class Zone
{
  public:
    Zone()
    {
      _i2c = new I2C();
      _i2c->setClockSpeed(I2CFAST);

      _oneWire = new OneWire(TEMPERATURE_SENSOR_PIN);
      _temperatureSensors = new DallasTemperature(_oneWire);

      _oneWire->reset();
      _temperatureSensors->begin();

      _useCelsius = true;

      _PID_target = 65.0;

      _temperatureSensor.push_back(new Temperature(_temperatureSensors, &temperatureSensorAddress[0]));
      _temperatureSensor.push_back(new Temperature(_temperatureSensors, &temperatureSensorAddress[1]));
      _temperatureSensor.push_back(new Temperature(_temperatureSensors, &temperatureSensorAddress[2]));
      _temperatureSensor.push_back(new Temperature(_temperatureSensors, &temperatureSensorAddress[3]));

      _PID = new PIDEnhanced(10, 0, 511, conservative_tune, aggressive_tune);

      _FanControl = new FanControl(_i2c, MAX31790_ADDR);

      if (DEBUG || VERBOSE)
      {
        Serial.println(_i2c->scan());
        Serial.println(listDevices(_temperatureSensors));
        Serial.println("Setup Complete");
      }
    }

    void periodic()
    {
      if (_useCelsius)
      {
        for (Temperature* sensor : _temperatureSensor)
          _temperature.push_back(sensor->getTemperatureC());
      }
      else
      {
        for (Temperature* sensor : _temperatureSensor)
          _temperature.push_back(sensor->getTemperatureF());
      }

      String PID_Profile = _PID->computeAvgOfVector(_temperature, _PID_target, &_PID_output);

      Serial.print("PID Profile: ");
      Serial.println(PID_Profile);

      Serial.print("PID Out: ");
      Serial.println(_PID_output);

      uint16_t tachRPM[NUM_FANS];
      uint8_t rv = _FanControl->getTachRPMComplete(tachRPM);

      //      Serial.print("rv: ");
      //      Serial.println(rv);
      //
      //      for (uint8_t idx = 0; idx < NUM_FANS; idx++)
      //      {
      //
      //        Serial.print("Fan ");
      //        Serial.print(idx);
      //        Serial.print(": ");
      //        Serial.println(tachRPM[idx]);
      //      }

      uint16_t pwmTargetOUT[NUM_PWM];
      rv = _FanControl->getPWMTargetComplete(pwmTargetOUT);

      //      Serial.print("rv: ");
      //      Serial.println(rv);
      //
      //      for (uint8_t idx = 0; idx < NUM_PWM; idx++)
      //      {
      //
      //        Serial.print("PWMOUT ");
      //        Serial.print(idx);
      //        Serial.print(": ");
      //        Serial.println(pwmTargetOUT[idx]);
      //      }

      uint16_t pwmDuty[NUM_PWM];
      rv = _FanControl->getPWMDutyComplete(pwmDuty);

      //      Serial.print("rv: ");
      //      Serial.println(rv);
      //
      //      for (uint8_t idx = 0; idx < NUM_PWM; idx++)
      //      {
      //
      //        Serial.print("PWMDUTY ");
      //        Serial.print(idx);
      //        Serial.print(": ");
      //        Serial.println(pwmDuty[idx]);
      //      }

      uint8_t pwmDutyPercent[NUM_PWM];
      rv = _FanControl->getPWMDutyPercentComplete(pwmDutyPercent);

      //      Serial.print("rv: ");
      //      Serial.println(rv);
      //
      //      for (uint8_t idx = 0; idx < NUM_PWM; idx++)
      //      {
      //
      //        Serial.print("PWMDUTYPercent ");
      //        Serial.print(idx);
      //        Serial.print(": ");
      //        Serial.println(pwmDutyPercent[idx]);
      //      }

      uint16_t pwmTargetIN[NUM_PWM] = {
        0,
        25,
        255,
        511
      };
      rv = _FanControl->setPWMTargetComplete(pwmTargetIN);

      //      Serial.print("rv: ");
      //      Serial.println(rv);

      rv = _FanControl->getPWMTargetComplete(pwmTargetOUT);

      //      Serial.print("rv: ");
      //      Serial.println(rv);
      //
      //      for (uint8_t idx = 0; idx < NUM_PWM; idx++)
      //      {
      //
      //        Serial.print("PWMOUT ");
      //        Serial.print(idx);
      //        Serial.print(": ");
      //        Serial.println(pwmTargetOUT[idx]);
      //      }
    }

  private:
    FanControl*     _FanControl;
    PIDEnhanced*   _PID;
    double   _PID_input;
    double   _PID_target;
    double   _PID_output;
    std::vector<double>   _temperature;
    std::vector<Temperature*> _temperatureSensor;
    bool           _useCelsius;
    I2C*                _i2c;
    OneWire*           _oneWire;
    DallasTemperature* _temperatureSensors;
};
#endif
