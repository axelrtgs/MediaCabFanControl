#pragma once

#include <PID_v1.h>

#ifndef PIDEnhanced_h
#define PIDEnhanced_h

class PIDEnhanced 
{
  private:
  PID_SETTINGS  _PIDSettings;
  PID           _PID;

  bool isWwithinTolerance() 
  {
    return abs(_PIDSettings.PIDSettings.target - _PIDSettings.input) < _PIDSettings.tolerance;
  }

  void setPIDTuningProfile() 
  {
    if (isWwithinTolerance())
    { 
      _PID.SetTunings(_PIDSettings.conservative.Kp, _PIDSettings.conservative.Ki, _PIDSettings.conservative.Kd);
      _PIDSettings.tuningProfile = "Conservative";
    }
    else
    {
      _PID.SetTunings(_PIDSettings.aggressive.Kp, _PIDSettings.aggressive.Ki, _PIDSettings.aggressive.Kd);
      _PIDSettings.tuningProfile = "Aggressive";
    }
  }

  public:
  struct PID_SETTINGS
  {
    double* input;
    double* output;
    double* target;
    uint8_t tolerance;
    int minDuty;
    int maxDuty;
    PID_TUNING conservative;
    PID_TUNING aggressive;
    String tuningProfile;
  };

  struct PID_TUNING
  {
    double Kp;
    double Ki;
    double Kd;
  };

  PIDEnhanced(PID_SETTINGS PIDSettings) 
  {
    _PIDSettings = PIDSettings
    _PID = PID(&_PIDSettings.input, &_PIDSettings.output, &_PIDSettings.PIDSettings.target, _PIDSettings.conservative.Kp, _PIDSettings.conservative.Ki, _PIDSettings.conservative.Kd, REVERSE);
    _PID.SetOutputLimits(_PIDSettings.minDuty, _PIDSettings.maxDuty);
    _PID.SetMode(AUTOMATIC);
  }

  void compute() 
  {
    setPIDTuningProfile();
    _PID.Compute();
  }
};
#endif
