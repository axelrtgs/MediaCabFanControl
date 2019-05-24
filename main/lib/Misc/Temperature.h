#pragma once

#include <DallasTemperature.h>

#ifndef Temperature_h
#define Temperature_h

// Outside of temperature class as it is called to list all devices on bus
inline String listDevices(DallasTemperature* sensors)
{
  int deviceCount;
  DeviceAddress tempDeviceAddress;
  String output;

  deviceCount = sensors->getDeviceCount();
  output.concat("\nNumber of devices: ");
  output.concat(deviceCount);

  for ( int i = 0; i < deviceCount; i++) {
    sensors->getAddress(tempDeviceAddress, i);
    output.concat("\nthe address for device ");
    output.concat(i);
    output.concat(" is: ");
    output.concat("  { ");
    for (uint8_t i = 0; i < 8; i++)
    {
      output.concat("0x");
      if (tempDeviceAddress[i] < 0x10) output.concat("0");
      output.concat(tempDeviceAddress[i]);
      if (i < 7) output.concat(", ");
    }
    output.concat(" }\n");
  }
  return output;
}

class Temperature
{
  public:
    Temperature(DallasTemperature* sensors, DeviceAddress* deviceAddress)
    {
      _sensors = sensors;
      _deviceAddress = deviceAddress;
    }

    inline void setResolution(int resolution)
    {
      _deviceAddress, _sensors->setResolution(resolution);
    }
    inline uint8_t getResolution()
    {
      _sensors->getResolution(*_deviceAddress);
    }

    inline void requestTemperatures()
    {
      _sensors->requestTemperaturesByAddress(*_deviceAddress);
    }
    inline double getTemperatureC()
    {
      return _current = _sensors->getTempC(*_deviceAddress);
    }
    inline double getTemperatureF()
    {
      return _current = _sensors->getTempF(*_deviceAddress);
    }

    inline void setTempLimits(double minimum, double maximum)
    {
      _minimum = minimum;
      _maximum = maximum;
    }
    inline bool isUpperLimit()
    {
      return _current > _maximum;
    }
    inline bool isLowerLimit()
    {
      return _current < _minimum;
    }
    inline bool hasAlarm()
    {
      return _sensors->hasAlarm(*_deviceAddress);
    }
    inline void setAlarmTempC(int8_t highTemp, int8_t lowTemp)
    {
      highTemp = std::min<int8_t>(highTemp, 125);
      _sensors->setHighAlarmTemp(*_deviceAddress, highTemp);

      lowTemp = std::max<int8_t>(-55, lowTemp);
      _sensors->setLowAlarmTemp(*_deviceAddress, lowTemp);
    }
    inline void getAlarmTempC(int8_t* highTemp, int8_t* lowTemp)
    {
      *highTemp = (_sensors->getHighAlarmTemp(*_deviceAddress));

      *lowTemp = (_sensors->getLowAlarmTemp(*_deviceAddress));
    }
    inline void setAlarmTempF(float highTempF, float lowTempF)
    {
      int8_t highTempC = _sensors->toCelsius(highTempF);
      highTempC = std::min<int8_t>(highTempC, 125);
      _sensors->setHighAlarmTemp(*_deviceAddress, highTempC);

      int8_t lowTempC = _sensors->toCelsius(lowTempF);
      lowTempC = std::max<int8_t>(-55, lowTempC);
      _sensors->setLowAlarmTemp(*_deviceAddress, lowTempC);
    }
    inline void getAlarmTempF(float* highTempF, float* lowTempF)
    {
      int8_t highTempC = _sensors->getHighAlarmTemp(*_deviceAddress);
      *highTempF = _sensors->toFahrenheit(highTempC);

      int8_t lowTempC = _sensors->getLowAlarmTemp(*_deviceAddress);
      *lowTempF = _sensors->toFahrenheit(lowTempC);
    }

  private:
    DallasTemperature* _sensors;
    DeviceAddress*  _deviceAddress;
    float     _current;
    float     _minimum;
    float     _maximum;
};
#endif
