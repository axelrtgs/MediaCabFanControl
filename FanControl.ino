#include <I2C.h>
#include <PWM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <avr/wdt.h>
#include "aREST.h"

const bool      DEBUG                         = false;
const bool      API                           = true;

const uint8_t   API_STATUS_PIN                = 13;

const uint8_t   MAX6651                       = 0x1B;     // https://datasheets.maximintegrated.com/en/ds/MAX6650-MAX6651.pdf

const uint16_t  WAIT                          = 1000;    // Amount of time (in ms) to wait between updates

const uint8_t   PWM_PIN                       = 10;       // PWM output pin
const uint16_t  PWM_FREQUENCY                 = 25000;   // PWM frequency (in Hz)
const uint8_t   PWM_MAX_DUTY                  = 255;      // PWM maximum duty value
const uint8_t   PWM_MIN_DUTY                  = 0;        // PWM minimum duty value
//PWM duty resulting in 0-0.8Vdc causes fan to stop P.9: http://partner.delta-corp.com/Products/FANUploads/Specification/AFC1212D-F00.pdf

const uint8_t   TACHOMETER_REVOLUTION_PULSES  = 2;
const uint8_t   TACHOMETER_COUNT_SECONDS      = 1;
const uint8_t   SECONDS_PER_MINUTE            = 60;

const uint16_t  MIN_FAN_RPM                   = 600;
const uint16_t  MAX_FAN_RPM                   = 4000;

const uint8_t   TEMP_SENSOR_PIN               = 7;
const uint8_t   MIN_FAN_TEMP                  = 22;
const uint8_t   MAX_FAN_TEMP                  = 25;
const uint8_t   TEMP_TOLERANCEC               = 5;
const uint8_t   TEMP_TOLERANCEF               = 10;

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(TEMP_SENSOR_PIN);

// Pass our oneWire reference to Dallas temperatureC.
DallasTemperature sensors(&oneWire);

//Define Variables we'll be connecting to
double target_tempC = 20.0, temperatureC = 0.0, target_tempF = 68.0, temperatureF = 0.0, PWM_DUTYC = 1, PWM_DUTYF = 1;
uint16_t tach0_rpm = 0, tach1_rpm = 0, tach2_rpm = 0, tach3_rpm = 0;
bool use_fahrenheight = false;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
PID myPIDC(&temperatureC, &PWM_DUTYC, &target_tempC, consKp, consKi, consKd, REVERSE);
PID myPIDF(&temperatureF, &PWM_DUTYF, &target_tempF, consKp, consKi, consKd, REVERSE);

aREST rest = aREST();

void printLine(const char  *text, String val) {
  Serial.print (text);
  Serial.println (val);
}

void writeData(uint8_t address, uint8_t registerAddress, uint8_t data) {
  uint8_t result = I2c.write(address, registerAddress, data);
  if (DEBUG) {
    printLine("Write result: ", String(result));
  }
}

void readData(uint8_t address, uint8_t registerAddress, uint8_t data) {
  uint8_t result = I2c.read(address, registerAddress, data);
  if (DEBUG) {
    printLine("Read result: ", String(result));
  }
}

uint16_t getRPMFromPulses(uint8_t pulses) {
  uint16_t pps = pulses / TACHOMETER_COUNT_SECONDS;
  uint16_t rps = pps / TACHOMETER_REVOLUTION_PULSES;
  uint16_t rpm = rps * SECONDS_PER_MINUTE;

  return rpm;
}

uint8_t getPWMDutyFromRPM(float rpm) {
  uint8_t rawDuty = ( rpm / MAX_FAN_RPM ) * PWM_MAX_DUTY;
  uint8_t maxDuty = min(rawDuty, PWM_MAX_DUTY); // Cap duty to max PWM value
  uint8_t minDuty = max(maxDuty, PWM_MIN_DUTY); // Cap duty to min PWM value
  return minDuty;
}

void setup() {
  // Start Serial port
  if (DEBUG || API) {
    Serial.begin(115200);
  }
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe();
  //sets the frequency for the specified pin
  SetPinFrequencySafe(PWM_PIN, PWM_FREQUENCY);
  //Set Startup PWM duty
  pwmWrite(PWM_PIN, 1);
  // Start up the temp sensor
  sensors.begin();
  // Initialize I2C bus
  I2c.begin();
  I2c.timeOut(500);
  // Disable internal pullup
  I2c.pullup(false);
  // Switch speed to 400khz
  I2c.setSpeed(true);
  // Scan bus for devices (for debug only)
  if (DEBUG) {
    I2c.scan();
  }
  // Write config register for MAX6651
  writeData(MAX6651, 0x02, 0x0A);
  //Read config register
  if (DEBUG) {
    readData(MAX6651, 0x02, 1);
    uint8_t configRB = I2c.receive();
    printLine ("Config Readback: ", String(configRB, HEX));
  }
  writeData(MAX6651, 0x00, getPWMDutyFromRPM(1000));
  
  //tell the PID to range between 0 and the full window size
  myPIDF.SetOutputLimits(PWM_MIN_DUTY, PWM_MAX_DUTY);
  //turn the PID on
  myPIDF.SetMode(AUTOMATIC);
  //tell the PID to range between 0 and the full window size
  myPIDC.SetOutputLimits(PWM_MIN_DUTY, PWM_MAX_DUTY);
  //turn the PID on
  myPIDC.SetMode(AUTOMATIC);  

  if (API) {
    rest.set_status_led(API_STATUS_PIN);
    rest.variable("temperatureC", &temperatureC);
    rest.variable("target_tempC", &target_tempC);
    rest.variable("temperatureC", &temperatureF);
    rest.variable("target_tempC", &target_tempF);
    rest.variable("pwm_dutyC", &PWM_DUTYC);
    rest.variable("pwm_dutyF", &PWM_DUTYF);
    rest.variable("use_fahrenheight", &use_fahrenheight);
    rest.variable("tach0_rpm", &tach0_rpm);
    rest.variable("tach1_rpm", &tach1_rpm);
    rest.variable("tach2_rpm", &tach2_rpm);
    rest.variable("tach3_rpm", &tach3_rpm);

    // Function to be exposed
    rest.function("settargettempC", set_target_tempC);
    rest.function("settargettempF", set_target_tempF);
    rest.function("setfahrenheight", set_fahrenheight);

    // Give name and ID to device (ID should be 6 characters long)
    rest.set_id("1");
    rest.set_name("FANCTL");

    // Start watchdog
    wdt_enable(WDTO_4S);
  }
  if (DEBUG || API) {
    printLine ("Setup ", "Complete");
  }
}

void loop() {
  if (API) {
    // Handle REST calls
    rest.handle(Serial);
    wdt_reset();
  }

  sensors.requestTemperatures(); // Send the command to get temperatures from all probes
  temperatureC = sensors.getTempCByIndex(0); //Get reading from first temperature sensor on
  temperatureF = sensors.getTempFByIndex(0); //Get reading from first temperature sensor on
  if (DEBUG) {
    printLine("Temp *C: ", String(temperatureC));
    printLine("Temp *F: ", String(temperatureF));
  }
  readData(MAX6651, 0x0C, 1);
  tach0_rpm = getRPMFromPulses(I2c.receive());

  if (DEBUG) {
    printLine("TACH0 RPM: ", String(tach0_rpm));
    printLine("TACH1 RPM: ", String(tach1_rpm));
    printLine("TACH2 RPM: ", String(tach2_rpm));
    printLine("TACH3 RPM: ", String(tach3_rpm));
  }
  double gapF = abs(target_tempF - temperatureF); //distance away from setpoint
  double gapC = abs(target_tempC - temperatureC); //distance away from setpoint
  if (gapC < TEMP_TOLERANCEC || gapF < TEMP_TOLERANCEF)
  { //we're close to setpoint, use conservative tuning parameters
    myPIDC.SetTunings(consKp, consKi, consKd);
    myPIDF.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPIDC.SetTunings(aggKp, aggKi, aggKd);
    myPIDF.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPIDF.Compute();
  myPIDC.Compute();
  
  if (use_fahrenheight) {
    pwmWrite(PWM_PIN, PWM_DUTYF);
  } else {
    pwmWrite(PWM_PIN, PWM_DUTYC);
  }

  if (DEBUG) {
    printLine("PIDC Compute Output: ", String(PWM_DUTYC));
    printLine("PIDF Compute Output: ", String(PWM_DUTYF));
  }
  delay(WAIT);

}

// Custom function accessible by the API
int set_target_tempC(String target_temp) {
  target_tempC = target_temp.toFloat();
}
int set_target_tempF(String target_temp) {
  target_tempF = target_temp.toFloat();
}

int set_fahrenheight(String fahrenheight) {
  use_fahrenheight = fahrenheight.toInt();
}
