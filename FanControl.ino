#include <I2C.h>
#include <PWM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

#define DEBUG    true

#define MAX6651  0x1B         // https://datasheets.maximintegrated.com/en/ds/MAX6650-MAX6651.pdf

#define WAIT 1000             // Amount of time (in ms) to wait between updates

#define TEMP_SENSOR_PIN 7     // Temperature sensor pin

#define PWM_PIN 10            // PWM output pin
#define PWM_FREQUENCY 25000   // PWM frequency (in Hz)
#define PWM_MAX_DUTY 255      // PWM maximum duty value
#define PWM_MIN_DUTY 0        // PWM minimum duty value
//PWM duty resulting in 0-0.8Vdc causes fan to stop P.9: http://partner.delta-corp.com/Products/FANUploads/Specification/AFC1212D-F00.pdf

#define TACHOMETER_REVOLUTION_PULSES 2
#define TACHOMETER_COUNT_SECONDS 1
#define SECONDS_PER_MINUTE 60

#define MIN_FAN_RPM 600
#define MAX_FAN_RPM 4000

#define MIN_FAN_TEMP 22
#define MAX_FAN_TEMP 25
#define TEMP_TOLERANCE 5

// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(TEMP_SENSOR_PIN);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

//Define Variables we'll be connecting to
double Setpoint, Temperature, PWM_DUTY;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Temperature, &PWM_DUTY, &Setpoint, consKp, consKi, consKd, REVERSE);

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
  uint8_t maxDuty = min(rawDuty,PWM_MAX_DUTY); // Cap duty to max PWM value
  uint8_t minDuty = max(maxDuty,PWM_MIN_DUTY); // Cap duty to min PWM value
  return minDuty;
}

void setup() {
  // Start Serial port
  if (DEBUG) {
    Serial.begin(9600);
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
  if (DEBUG) {
    printLine ("Setup ", "Complete");
  }
  Setpoint = 20.0;
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(PWM_MIN_DUTY, PWM_MAX_DUTY);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  sensors.requestTemperatures(); // Send the command to get temperatures from all probes
  Temperature = sensors.getTempCByIndex(0); //Get reading from first temperature sensor on 
  if (DEBUG) {
    printLine("Temp *C: ", String(Temperature));
  }
  readData(MAX6651, 0x0C, 1);
  uint8_t tach_pulses = I2c.receive();
  if (DEBUG) {
    printLine("TACH0 RPM: ", String(getRPMFromPulses(tach_pulses)));
  }
  double gap = abs(Setpoint-Temperature); //distance away from setpoint
  if (gap < TEMP_TOLERANCE)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();

  pwmWrite(PWM_PIN, PWM_DUTY);
  
  if (DEBUG) {
    printLine("PID Compute Output: ", String(PWM_DUTY));
  }
  delay(WAIT);

}
