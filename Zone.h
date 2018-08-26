#pragma once

#include "MAX6651.h"
#include "PWM.h"
#include "PIDEnhanced.h"
#include "Temperature.h"
#include "Utilities.h"

#ifndef Zone_h
#define Zone_h

class Zone 
{
    private:
    MAX6651     _MAX6651;
    INFO        _info;
    PIDEnhanced _PID;
    PWM         _PWM;
    TEMPERATURE _temperature;

    public:
    struct INFO
    {
        TACH            tach;
        PID_SETTINGS    PIDSettings;
        int             MAX6651Address;
        int             PWMPin;
        int             temperaturePin;
        DeviceAddress   temperatureAddress;
    };

    Zone(INFO info)
    {
        _info = info;
        _MAX6651 = MAX6651(_info.MAX6651Address);

        _PWM = PWM(PWMPin);
        _info.PIDSettings.maxDuty = _PWM.getMaxValue();
        
        _temperature = Temperature(temperaturePin, temperatureAddress);
        _PID = PIDEnhanced(_info.PIDSettings);
    }

    periodic()
    {
        _info.tach = _MAX6651.getTach();
        _PID.compute();
        _PWM.setValue(_info.PIDSettings.output);
    }
};
#endif
