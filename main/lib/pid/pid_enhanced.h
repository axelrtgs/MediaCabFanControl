#pragma once

#ifndef PIDEnhanced_h
#define PIDEnhanced_h

#include <pid.h>

class PIDEnhanced {
 public:
  struct PID_TUNING {
    double Kp;
    double Ki;
    double Kd;
  };

  PIDEnhanced(const int &tolerance, const int &bang_on, const int &bang_off,
              const int &minDuty, const int &maxDuty,
              const PID_TUNING &conservative, const PID_TUNING &aggressive) {
    _tolerance = tolerance;
    _conservative = conservative;
    _aggressive = aggressive;

    _PID =
        new PID(&_input, &_target, &_output, minDuty, maxDuty, _conservative.Kp,
                _conservative.Ki, _conservative.Kd, P_ON_E, REVERSE);
    _PID->setBangBang(bang_on, bang_off);
    _PID->setTimeStep(5000);
  }

  inline std::string compute(const double &input, const double &target,
                             double *output) {
    _input = input;
    _target = target;

    setPIDTuningProfile();

    _PID->run();

    *output = _output;

    return _tuningProfile;
  }

 private:
  PID_TUNING _conservative;
  PID_TUNING _aggressive;
  int _tolerance;
  PID *_PID;
  std::string _tuningProfile;
  double _input;
  double _target;
  double _output;

  inline bool isWwithinTolerance() {
    return abs(_target - _input) < _tolerance;
  }

  inline void setPIDTuningProfile() {
    if (isWwithinTolerance()) {
      _PID->setGains(_conservative.Kp, _conservative.Ki, _conservative.Kd);
      _tuningProfile = "Conservative";
    } else {
      _PID->setGains(_aggressive.Kp, _aggressive.Ki, _aggressive.Kd);
      _tuningProfile = "Aggressive";
    }
  }
};
#endif
