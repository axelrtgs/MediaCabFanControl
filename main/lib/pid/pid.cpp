#include "pid.h"

#include <utilities.h>

#include "esp_timer.h"
namespace PID {
PID::PID(double *input, double *setpoint, double *output, double outputMin,
         double outputMax, double Kp, double Ki, double Kd, PROPORTIONAL pOn,
         DIRECTION controllerDirection) {
  _input = input;
  _setpoint = setpoint;
  _output = output;
  SetOutputLimits(outputMin, outputMax);
  setGains(Kp, Ki, Kd, pOn);
  SetControllerDirection(controllerDirection);
  _timeStep = SECOND_IN_USEC;
}

void PID::SetOutputLimits(double min, double max) {
  if (min >= max) return;
  _outputMin = min;
  _outputMax = max;

  if (*_output > _outputMax)
    *_output = _outputMax;
  else if (*_output < _outputMin)
    *_output = _outputMin;

  if (_outputSum > _outputMax)
    _outputSum = _outputMax;
  else if (_outputSum < _outputMin)
    _outputSum = _outputMin;
}

void PID::setGains(double Kp, double Ki, double Kd, PROPORTIONAL pOn) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  _pOn = pOn;
  _pOnE = pOn == PROPORTIONAL::ON_ERROR;
  double sampleTimeInSec = _timeStep / SECOND_IN_USEC;
  _Kp = Kp;
  _Ki = Ki * sampleTimeInSec;
  _Kd = Kd / sampleTimeInSec;

  if (_controllerDirection == DIRECTION::REVERSE) {
    _Kp = (0 - _Kp);
    _Ki = (0 - _Ki);
    _Kd = (0 - _Kd);
  }
}

void PID::setGains(double Kp, double Ki, double Kd) {
  setGains(Kp, Ki, Kd, _pOn);
}

void PID::setBangBang(double bangOn, double bangOff) {
  _bangOn = bangOn;
  _bangOff = bangOff;
}

void PID::setBangBang(double bangRange) { setBangBang(bangRange, bangRange); }

void PID::setTimeStep(int64_t timeStep) {
  if (timeStep > 0) {
    double ratio = (double)timeStep / (double)_timeStep;
    _Ki *= ratio;
    _Kd /= ratio;
    _timeStep = timeStep;
  }
}

bool PID::atSetPoint(double threshold) {
  return abs(*_setpoint - *_input) <= threshold;
}

void PID::run() {
  if (_stopped) {
    _stopped = false;
    reset();
  }

  uint64_t now = esp_timer_get_time();

  if (_bangOn && ((*_setpoint - *_input) > _bangOn)) {
    *_output = _outputMax;
    _lastStep = now;
  } else if (_bangOff && ((*_input - *_setpoint) > _bangOff)) {
    *_output = _outputMin;
    _lastStep = now;
  } else {
    if (now - _lastStep >= _timeStep) {
      _lastStep = now;
      double error = *_setpoint - *_input;
      double dInput = *_input - _previousInput;
      _outputSum += _Ki * error;

      if (!_pOnE) _outputSum -= _Kp * dInput;

      _outputSum = Utilities::clamp_val(_outputSum, _outputMin, _outputMax);

      double output = _pOnE ? _Kp * error : 0;

      output += _outputSum - _Kd * dInput;

      *_output = Utilities::clamp_val(output, _outputMin, _outputMax);
      _previousInput = *_input;
    }
  }
}

void PID::SetControllerDirection(DIRECTION Direction) {
  if (Direction != _controllerDirection) {
    _Kp = (0 - _Kp);
    _Ki = (0 - _Ki);
    _Kd = (0 - _Kd);
  }
  _controllerDirection = Direction;
}

void PID::stop() {
  _stopped = true;
  reset();
}

void PID::reset() {
  _lastStep = esp_timer_get_time();
  _outputSum = 0;
  _previousInput = 0;
}

bool PID::isStopped() { return _stopped; }
}  // namespace PID
