#include <lib/Misc/Utilities.h>
#include "PID.h"

PID::PID(double *input, double *setpoint, double *output, double outputMin, double outputMax,
                 double Kp, double Ki, double Kd) {
    _input = input;
    _setpoint = setpoint;
    _output = output;
    _outputMin = outputMin;
    _outputMax = outputMax;
    setGains(Kp, Ki, Kd);
    _timeStep = 1000000;//1 second
}

void PID::setGains(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void PID::setBangBang(double bangOn, double bangOff) {
    _bangOn = bangOn;
    _bangOff = bangOff;
}

void PID::setBangBang(double bangRange) {
    setBangBang(bangRange, bangRange);
}

void PID::setOutputRange(double outputMin, double outputMax) {
    _outputMin = outputMin;
    _outputMax = outputMax;
}

void PID::setTimeStep(int64_t timeStep){
    _timeStep = timeStep;
}


bool PID::atSetPoint(double threshold) {
    return abs(*_setpoint - *_input) <= threshold;
}

void PID::run() {
    if (_stopped) {
        _stopped = false;
        reset();
    }
    //if bang thresholds are defined and we're outside of them, use bang-bang control
    if (_bangOn && ((*_setpoint - *_input) > _bangOn)) {
        *_output = _outputMax;
        _lastStep = esp_timer_get_time();
    } else if (_bangOff && ((*_input - *_setpoint) > _bangOff)) {
        *_output = _outputMin;
        _lastStep = esp_timer_get_time();
    } else {                                    //otherwise use PID control
        unsigned long _dT = esp_timer_get_time() - _lastStep;   //calculate time since last update
        if (_dT >= _timeStep) {                     //if long enough, do PID calculations
            _lastStep = esp_timer_get_time();
            double _error = *_setpoint - *_input;
            _integral += (_error + _previousError) / 2 * _dT / 1000.0;   //Riemann sum integral
            //_integral = constrain(_integral, _outputMin/_Ki, _outputMax/_Ki);
            double _dError = (_error - _previousError) / _dT / 1000.0;   //derivative
            _previousError = _error;
            double PID = (_Kp * _error) + (_Ki * _integral) + (_Kd * _dError);
            //*_output = _outputMin + (constrain(PID, 0, 1) * (_outputMax - _outputMin));
            *_output = clamp(PID, _outputMin, _outputMax);
        }
    }
}

void PID::stop() {
    _stopped = true;
    reset();
}
void PID::reset() {
    _lastStep = esp_timer_get_time();
    _integral = 0;
    _previousError = 0;
}

bool PID::isStopped(){
    return _stopped;
}

double PID::getIntegral(){
    return _integral;
}

void PID::setIntegral(double integral){
    _integral = integral;
}

void PIDRelay::run() {
    PID::run();
    while ((esp_timer_get_time() - _lastPulseTime) > _pulseWidth) _lastPulseTime += _pulseWidth;
    *_relayState = ((esp_timer_get_time() - _lastPulseTime) < (_pulseValue * _pulseWidth));
}


double PIDRelay::getPulseValue(){
    return (isStopped()?0:_pulseValue);
}
