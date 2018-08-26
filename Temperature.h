#pragma once

#include <OneWire.h>
#include <DallasTemperature.h>

#ifndef Temperature_h
#define Temperature_h

class Temperature 
{
  private:
  DeviceAddress _deviceAddress;
  double  _current;
  double  _minimum;
  double  _maximum;
  bool    _use_fahrenheight;
  const int DEFAULT_RESOLUTION = 9;

  public:
  Temperature(int pin, DeviceAddress deviceAddress) 
  : OneWire oneWire(pin), 
  DallasTemperature sensors(&oneWire) 
  {
    _deviceAddress = deviceAddress;
    sensors.begin();
    setResolution(DEFAULT_RESOLUTION)
  }

  void setTempLimits(double min, double max) 
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
    sensors.setResolution(resolution);
  }

  double getTemperature() 
  {
    if(_use_fahrenheight)
      _current = sensors.getTempF(_deviceAddress);
    else
      _current = sensors.getTempC(_deviceAddress);
    return _current;
  }
};
#endif
