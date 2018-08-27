#pragma once

#include <OneWire.h>
#include <DallasTemperature.h>

#ifndef Temperature_h
#define Temperature_h

class Temperature
{
  private:
    DallasTemperature* _sensors;
    uint8_t* _deviceAddress;
    double  _current;
    double  _minimum;
    double  _maximum;
    bool    _use_fahrenheight;
    const int DEFAULT_RESOLUTION = 9;

    void printAddress(DeviceAddress deviceAddress) {
      Serial.print("  { ");
      for (uint8_t i = 0; i < 8; i++)
      {
        Serial.print("0x");
        if (deviceAddress[i] < 0x10) Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
        if (i < 7) Serial.print(", ");
      }
      Serial.println(" }");
    }

  public:
    Temperature(DallasTemperature* sensors)
    {
      _sensors = sensors;
      _sensors->begin();
      setResolution(DEFAULT_RESOLUTION);
    }

    void setAddress(DeviceAddress deviceAddress)
    {
      _deviceAddress = deviceAddress;
    }

    void setTempLimits(double minimum, double maximum)
    {
      _minimum = minimum;
      _maximum = maximum;
    }

    void useFarenheight(bool use_fahrenheight)
    {
      _use_fahrenheight = use_fahrenheight;
    }

    bool isUpperLimit()
    {
      return _current > _maximum;
    }

    bool isLowerLimit()
    {
      return _current < _minimum;
    }

    void setResolution(int resolution)
    {
      _sensors->setResolution(resolution);
    }

    double getTemperature()
    {
      if (_use_fahrenheight)
        _current = _sensors->getTempF(_deviceAddress);
      else
        _current = _sensors->getTempC(_deviceAddress);
      return _current;
    }

    void listDevices()
    {
      int deviceCount;
      DeviceAddress tempDeviceAddress;

      deviceCount = _sensors->getDeviceCount();
      Serial.print("\nNumber of devices: ");
      Serial.println(deviceCount);

      for ( int i = 0; i < deviceCount; i++) {
        _sensors->getAddress(tempDeviceAddress, i);
        Serial.print("\nthe address for device ");
        Serial.print(i);
        Serial.print(" is: ");
        printAddress(tempDeviceAddress);
      }
    }
};
#endif
