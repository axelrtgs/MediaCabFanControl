#pragma once

#include "I2C.h"

#ifndef MAX6651_h
#define MAX6651_h

class MAX6651 
{
    private:
        int _deviceAddress;
        const uint8_t CONFIG = 0x0A; // https://datasheets.maximintegrated.com/en/ds/MAX6650-MAX6651.pdf
        const uint8_t SECONDS_PER_MINUTE            = 60;
        const uint8_t TACHOMETER_COUNT_SECONDS      = 1;
        const uint8_t TACHOMETER_REVOLUTION_PULSES  = 2;
        const int     BUS_FREQUENCY                 = 400000; // I2C fast mode 40mhz
        const int     CONFIG_REGISTER               = 0x02;
        const int     FAN1_RPM_REGISTER             = 0x0C;
        const int     FAN2_RPM_REGISTER             = 0x0E;
        const int     FAN3_RPM_REGISTER             = 0x10;
        const int     FAN4_RPM_REGISTER             = 0x12;
        const uint8_t FAN_REGISTER_SIZE             = 1;

    public:
    struct TACH
    {
        uint16_t fan1_rpm;
        uint16_t fan2_rpm;
        uint16_t fan3_rpm;
        uint16_t fan4_rpm;
    };
    
    MAX6651(int deviceAddress) 
    : I2C(BUS_FREQUENCY) 
    {
        _deviceAddress = deviceAddress;
        I2C.writeData(_deviceAddress, CONFIG_REGISTER, CONFIG);
    }

    uint16_t getRPMFromPulses(uint8_t pulses) 
    {
        return ((pulses / TACHOMETER_COUNT_SECONDS) / TACHOMETER_REVOLUTION_PULSES) * SECONDS_PER_MINUTE;
    }

    TACH getTach() 
    {
        TACH tach;
        tach.fan1_rpm = getRPMFromPulses(I2C.readData(_deviceAddress, FAN1_RPM_REGISTER, FAN_REGISTER_SIZE).toInt());
        tach.fan2_rpm = getRPMFromPulses(I2C.readData(_deviceAddress, FAN2_RPM_REGISTER, FAN_REGISTER_SIZE).toInt());
        tach.fan3_rpm = getRPMFromPulses(I2C.readData(_deviceAddress, FAN3_RPM_REGISTER, FAN_REGISTER_SIZE).toInt());
        tach.fan4_rpm = getRPMFromPulses(I2C.readData(_deviceAddress, FAN4_RPM_REGISTER, FAN_REGISTER_SIZE).toInt());
        return tach;
    }

    void scanBusAddresses() 
    {
        I2C.scan();
    }
};
#endif
