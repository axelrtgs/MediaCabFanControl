#pragma once
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiMDNSResponder.h>
#include <ArduinoJson.h>
#include "TinyWebServer.h"

#ifndef WifiControl_h
#define WifiControl_h

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
const char ssid[] = "WiFi";        // your network SSID (name)
const char pass[] = "idontremembermypassword";    // your network password (use for WPA, or use as key for WEP)

const char mdnsName[] = "cabinetfan"; // the MDNS name that the board will respond to
// Note that the actual MDNS name will have '.local' after

const size_t BUFFER_SIZE_IN  = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + 90;
const size_t BUFFER_SIZE_OUT = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(5);

char json[BUFFER_SIZE_IN];

// Create a MDNS responder to listen and respond to MDNS name requests.
WiFiMDNSResponder mdnsResponder;

template<typename T> void serialize_JSON(T client, bool pretty = false)
{
  DynamicJsonBuffer jsonBuffer(BUFFER_SIZE_OUT);

  JsonObject& root = jsonBuffer.createObject();
  JsonObject& tach = root.createNestedObject("tach");
  //tach["tach0_rpm"] = tach_data.tach0_rpm;
  //tach["tach1_rpm"] = tach_data.tach1_rpm;
  //tach["tach2_rpm"] = tach_data.tach2_rpm;
  //tach["tach3_rpm"] = tach_data.tach3_rpm;

  JsonObject& temp = root.createNestedObject("temp");
  //temp["temperature"] = temp_data.temperature;
  //temp["target_temp"] = temp_data.target_temp;
  //temp["use_fahrenheight"] = temp_data.use_fahrenheight;

  JsonObject& pwm = root.createNestedObject("pwm");
  //pwm["pwm_duty"] = pwm_duty;

  if (!pretty)
    root.printTo(client);
  else
    root.prettyPrintTo(client);
}

bool deserialize_JSON(String json)
{
  if (DEBUG)
    Serial.println(json);

  DynamicJsonBuffer jsonBuffer(BUFFER_SIZE_IN);

  JsonObject& root = jsonBuffer.parseObject(json);
  if (!root.success())
  {
    Serial.println(F("JSON parsing failed!"));
    return false;
  }
  JsonObject& temp = root["temp"];
  if (!temp.success())
  {
    Serial.println("Temperature parsing failed!");
    return false;
  }

  JsonVariant target_temp = temp["target_temp"];
  JsonVariant use_fahrenheight = temp["use_fahrenheight"];

  if (target_temp.success() && use_fahrenheight.success())
  {
    //temp_data.target_temp      = target_temp.as<double>();
    //temp_data.use_fahrenheight  = use_fahrenheight.as<bool>();
    return true;
  }
  else
  {
    Serial.println("Temperature Fields parsing failed!!");
    return false;
  }
}

boolean index_handler(TinyWebServer& web_server);
boolean post_handler(TinyWebServer& web_server);
boolean get_handler(TinyWebServer& web_server);

TinyWebServer::PathHandler handlers[] =
{
  {"/", TinyWebServer::GET, &index_handler },
  {"/get", TinyWebServer::GET, &get_handler },
  {"/post", TinyWebServer::POST, &post_handler },
  {NULL},
};

boolean index_handler(TinyWebServer& web_server)
{
  web_server.send_error_code(200);
  web_server.end_headers();
  web_server.println("<!DOCTYPE HTML>");
  web_server.println("<html>");
  web_server.println("<pre>");

  serialize_JSON(web_server, true);

  web_server.println("</pre>");
  web_server.println("</html>");
  return true;
}

boolean get_handler(TinyWebServer& web_server)
{
  web_server.send_error_code(200);
  web_server.end_headers();
  web_server.send_content_type("application/json");

  serialize_JSON(web_server);
  return true;
}

boolean post_handler(TinyWebServer& web_server)
{
  int ndx = 0;
  Client& client = web_server.get_client();
  while (client.available())
  {
    char ch = client.read();
    json[ndx] = ch;
    ndx++;
  }

  if (deserialize_JSON(json))
    web_server.send_error_code(200);
  else
    web_server.send_error_code(500);

  web_server.end_headers();
  web_server.send_content_type("application/json");
  serialize_JSON(web_server);

  return true;
}

TinyWebServer web = TinyWebServer(handlers, NULL);

void printWiFiStatus()
{
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength in dBm (RSSI): ");
  Serial.println(WiFi.RSSI());
}

void WaitForWifiShielBlocking()
{
  while (WiFi.status() == WL_NO_SHIELD)
    Serial.println("WiFi shield not present");
}

void ConnectToWIfiBlocking()
{
  int status = WL_IDLE_STATUS;
  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    if (DEBUG || VERBOSE)
    {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
    }
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
}

void initializeWifi()
{
  WaitForWifiShielBlocking();
  ConnectToWIfiBlocking();
  if (DEBUG || VERBOSE)
    printWiFiStatus();

  // Setup the MDNS responder to listen to the configured name.
  // NOTE: You _must_ call this _after_ connecting to the WiFi network and
  // being assigned an IP address.
  if (!mdnsResponder.begin(mdnsName))
    Serial.println("Failed to start MDNS responder!");

  if (DEBUG || VERBOSE)
  {
    Serial.print("Server listening at http://");
    Serial.print(mdnsName);
    Serial.println(".local: ");
  }
  web.begin();
}

void WaitForWifi()
{
  // Call the update() function on the MDNS responder every loop iteration to
  // make sure it can detect and respond to name requests.
  mdnsResponder.poll();

  web.process();
}

#endif
