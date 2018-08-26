//Must be first two lines so includes can pick them up.
const bool  DEBUG   = true;
const bool  VERBOSE = true;

#include "WifiControl.h"
#include "Zone.h"


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

void setup() {
  // Start Serial port
  Serial.begin(9600);
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  if (DEBUG || VERBOSE) 
    printLine ("Setup Complete");
}

void loop() {

}

