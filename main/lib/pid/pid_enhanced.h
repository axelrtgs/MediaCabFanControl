#pragma once

#ifndef PIDEnhanced_h
#define PIDEnhanced_h

#include <pid.h>

#include <string>

namespace PID {
typedef struct tuning {
  double Kp;
  double Ki;
  double Kd;
} tuning_t;

class PIDEnhanced {
 public:
  PIDEnhanced(const double &tolerance, const double &bang_on,
              const double &bang_off, const double &minDuty,
              const double &maxDuty, const tuning_t &conservative,
              const tuning_t &aggressive) {
    _tolerance = tolerance;
    _conservative = conservative;
    _aggressive = aggressive;

    _PID = new PID(&_input, &_target, &_output, minDuty, maxDuty,
                   _conservative.Kp, _conservative.Ki, _conservative.Kd,
                   PROPORTIONAL::ON_ERROR, DIRECTION::REVERSE);
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
  tuning_t _conservative;
  tuning_t _aggressive;
  double _tolerance;
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
}  // namespace PID
#endif
