#pragma once

#include <Wire.h>
#include "Utilities.h"

#ifndef I2C_h
#define I2C_h

class I2C
{
  private:
    int _frequency;

  public:
    I2C(int freq)
    {
      Wire.begin();
      setClock(freq);
    }

    void setClock(int freq)
    {
      _frequency = freq;
      Wire.setClock(_frequency);
    }

    int getClock()
    {
      return _frequency;
    }

    void scan()
    {
      printLine("Begin I2C device scan");
      for (uint8_t addr = 8; addr <= 250; addr++)
      {
        Wire.beginTransmission(addr);
        Wire.write((uint8_t)0);
        if (Wire.endTransmission() == 0)
          printLine("Found I2C device at address: ", String(addr));
      }
      printLine("End I2C device scan");
    }

    void writeData(uint8_t address, uint8_t registerAddress, uint8_t data)
    {
      Wire.beginTransmission(address);
      Wire.write(byte(registerAddress));
      Wire.write(byte(data));
      Wire.endTransmission();
    }

    String readData(uint8_t address, uint8_t registerAddress, uint8_t numBytes)
    {
      Wire.beginTransmission(address);
      Wire.write(byte(registerAddress));
      Wire.endTransmission();

      Wire.requestFrom(address, numBytes);

      String content = "";

      while (Wire.available())
      {
        char character = Wire.read();
        content.concat(character);
      }
      return content;
    }
};
#endif
