#pragma once

#ifndef PID_H
#define PID_H

#include <stdint.h>

namespace {
const float SECOND_IN_USEC = 1000000.0;
}

namespace PID {
enum class DIRECTION : const uint8_t { DIRECT = 0, REVERSE = 1 };
enum class PROPORTIONAL : const uint8_t { ON_MEASUREMENT = 0, ON_ERROR = 1 };

class PID {
 public:
  PID(double *input, double *setpoint, double *output, double outputMin,
      double outputMax, double Kp, double Ki, double Kd, PROPORTIONAL pOn,
      DIRECTION controllerDirection);
  void SetOutputLimits(double Min, double Max);
  void setGains(double Kp, double Ki, double Kd, PROPORTIONAL pOn);
  void setGains(double Kp, double Ki, double Kd);
  void setBangBang(double bangOn, double bangOff);
  void setBangBang(double bangRange);
  void setTimeStep(int64_t timeStep);
  bool atSetPoint(double threshold);
  void run();
  void stop();
  void reset();
  bool isStopped();
  void SetControllerDirection(DIRECTION Direction);

 private:
  double _Kp, _Ki, _Kd;
  double _previousInput;
  double _bangOn, _bangOff;
  double *_input, *_setpoint, *_output;
  double _outputMin, _outputMax, _outputSum;
  int64_t _timeStep, _lastStep;
  DIRECTION _controllerDirection;
  PROPORTIONAL _pOn;
  bool _stopped, _pOnE;
};
}  // namespace PID
#endif
