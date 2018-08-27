//Must be first two lines so includes can pick them up.
const bool  DEBUG   = true;
const bool  VERBOSE = true;

#include "I2C.h"
#include "MAX6651.h"
#include "Temperature.h"
#include "Utilities.h"


//Pins 4,5,7,10 used by WiFi and SD module.
//PWM duty resulting in 0-0.8Vdc causes fan to stop P.9: http://partner.delta-corp.com/Products/FANUploads/Specification/AFC1212D-F00.pdf

// conservative_tune{
//       Kp = 1.0;
//       Ki = 0.05;
//       Kd = 0.25;
//     };
//     aggressive_tune{
//       Kp = 4.0;
//       Ki = 0.2;
//       Kd = 1.0;
//     };

// Found I2C device at address: 27
// Found I2C device at address: 155

const int TEMPERATURE_SENSOR_PIN = 2;
const int I2C_BUS_SPEED = 400000;
//DeviceAddress sensorAddress = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
DeviceAddress sensorAddress = { 0x28, 0x97, 0x23, 0xE5, 0x8, 0x0, 0x0, 0x71 };

MAX6651* max6651;
Temperature* temperature;


OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

void setup() {
  // Start Serial port
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  I2C i2c(I2C_BUS_SPEED);
  max6651 = new MAX6651(&i2c, 155);
  temperature = new Temperature(&sensors);

  if (DEBUG || VERBOSE)
  {
    i2c.scan();
    temperature->listDevices();
  }

  temperature->setAddress(sensorAddress);

  if (DEBUG || VERBOSE)
    printLine ("Setup Complete");
}

void loop() {
  //MAX6651::TACH tach;
  //tach = max6651->getTach();

  //printLine ("TACH1: ", String(tach.fan1_rpm));
  //printLine ("TACH2: ", String(tach.fan2_rpm));
  //printLine ("TACH3: ", String(tach.fan3_rpm));
  //printLine ("TACH4: ", String(tach.fan4_rpm));
  printLine ("Temp: ", String(temperature->getTemperature()));


}




