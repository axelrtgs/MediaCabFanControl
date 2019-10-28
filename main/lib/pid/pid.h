#pragma once

#include "esp_timer.h"

#ifndef PID_H
#define PID_H

class PID
{
public:
    #define DIRECT 0
    #define REVERSE 1
    #define P_ON_M 0
    #define P_ON_E 1
    #define SECOND 1000000.0

    PID(double *input, double *setpoint, double *output, double outputMin, double outputMax,
            double Kp, double Ki, double Kd, int pOn, int controllerDirection);
    void SetOutputLimits(double Min, double Max);
    void setGains(double Kp, double Ki, double Kd, int pOn);
    void setGains(double Kp, double Ki, double Kd);void setBangBang(double bangOn, double bangOff);
    void setBangBang(double bangRange);
    void setTimeStep(int64_t timeStep);
    bool atSetPoint(double threshold);
    void run();
    void stop();
    void reset();
    bool isStopped();
    void SetControllerDirection(int Direction);

private:
    double _Kp, _Ki, _Kd;
    double _previousInput;
    double _bangOn, _bangOff;
    double *_input, *_setpoint, *_output;
    double _outputMin, _outputMax, _outputSum;
    int64_t _timeStep, _lastStep;
    int _controllerDirection;
    int _pOn;
    bool _stopped, _pOnE;
};
#endif