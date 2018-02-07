#include <ArduinoJson.h>

struct TACH {
   uint16_t tach0_rpm = 0;
   uint16_t tach1_rpm = 0;
   uint16_t tach2_rpm = 0;
   uint16_t tach3_rpm = 0;
} tach_data;

struct TEMP {
   double temperatureC = 0.0;
   double temperatureF = 0.0;
   double target_tempC = 0.0;
   double target_tempF = 0.0;
   bool use_fahrenheight = false;
} temp_data;

struct PWM {
  double pwm_dutyC = 1.0;
  double pwm_dutyF = 1.0;
} pwm_data;

void setup() {
  Serial.begin(9600);
}

void loop() {
//NOOP
}

void serialEvent() {
  if (Serial.available()) {
    String json = Serial.readStringUntil('\n');
    if (deserialize_JSON(tach_data, temp_data, pwm_data, json)) {
      Serial.print("Success parsing JSON: ");
      Serial.println(json);
    } else {
      Serial.print("Failed to parse JSON: ");
      Serial.println(json);
    }
  }
}

bool deserialize_JSON(TACH& tach_data, TEMP& temp_data, PWM& pwm_data, String json)
{
    const size_t bufferSize = JSON_OBJECT_SIZE(2) + 2*JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(5) + 240;
    DynamicJsonBuffer jsonBuffer(bufferSize);

    JsonObject& root = jsonBuffer.parseObject(json);
    
    const char* command = root["command"]; // "set"
    Serial.println(command);
    if (command == "get") {
      serialize_JSON(tach_data, temp_data, pwm_data);
    } else if (command == "set") {
      JsonObject& tach = root["tach"];
      tach_data.tach0_rpm         = tach["tach0_rpm"]; // "1000"
      tach_data.tach1_rpm         = tach["tach1_rpm"]; // "2000"
      tach_data.tach2_rpm         = tach["tach2_rpm"]; // "3000"
      tach_data.tach3_rpm         = tach["tach3_rpm"]; // "4000"
      
      JsonObject& temp = root["temp"];
      temp_data.temperatureC      = temp["temperatureC"]; // "30.0"
      temp_data.temperatureF      = temp["temperatureF"]; // "79.8"
      temp_data.target_tempC      = temp["target_tempC"]; // "30.0"
      temp_data.target_tempF      = temp["target_tempF"]; // "79.8"
      temp_data.use_fahrenheight  = temp["use_fahrenheight"]; // false
      
      pwm_data.pwm_dutyC          = root["pwm"]["pwm_dutyC"]; // "128.0"
      pwm_data.pwm_dutyF          = root["pwm"]["pwm_dutyF"]; // "255.0"
  
      return root.success();
    } else {
      Serial.print("Invalid API command: ");
      Serial.println(command);
      return false;
    }
}

void serialize_JSON(TACH& tach_data, TEMP& temp_data, PWM& pwm_data) {
  const size_t bufferSize = JSON_OBJECT_SIZE(2) + 2*JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(5);
  DynamicJsonBuffer jsonBuffer(bufferSize);
  
  JsonObject& root = jsonBuffer.createObject();
  root["command"] = "push";
  
  JsonObject& tach = root.createNestedObject("tach");
  tach["tach0_rpm"] = tach_data.tach0_rpm;
  tach["tach1_rpm"] = tach_data.tach1_rpm;
  tach["tach2_rpm"] = tach_data.tach2_rpm;
  tach["tach3_rpm"] = tach_data.tach3_rpm;
  
  JsonObject& temp = root.createNestedObject("temp");
  temp["temperatureC"] = temp_data.temperatureC;
  temp["temperatureF"] = temp_data.temperatureF;
  temp["target_tempC"] = temp_data.target_tempC;
  temp["target_tempF"] = temp_data.target_tempF;
  temp["use_fahrenheight"] = temp_data.use_fahrenheight;
  
  JsonObject& pwm = root.createNestedObject("pwm");
  pwm["pwm_dutyC"] = pwm_data.pwm_dutyC;
  pwm["pwm_dutyF"] = pwm_data.pwm_dutyF;

  Serial.println(tach_data.tach0_rpm);
  
  root.printTo(Serial);
}
