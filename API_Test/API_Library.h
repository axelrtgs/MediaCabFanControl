#ifndef API_Library_h
#define API_Library_h

class API_Library {

enum API_TYPE { UNDEFINED, JSON, REST };

API_TYPE api_call_type      = UNDEFINED;
String   api_input          = "";
bool     api_input_complete = false;

public:

void setup() {
  Serial.begin(9600);
  api_input.reserve(200);
}

void loop() {
  if (api_input_complete) {
    Serial.println(api_input);

    reset_state(false);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char) Serial.read();

    switch (api_call_type) {
      case UNDEFINED:
        detect_api_type(inChar);
      case JSON:
        api_input += inChar;
        if (inChar == '}') {
          api_input += '\0';
          handle_json_msg(api_input);
          reset_state(true);
        }
      case REST:
        if (inChar == '\n') {
          api_input += '\0';
          handle_rest_msg(api_input);
          reset_state(true);
        } else {
          api_input += inChar;
        }
    }
  }
}

private:

void reset_state(bool input_complete) {
  api_call_type           = UNDEFINED;
  api_input_complete      = input_complete;
  if (!input_complete) {
    api_input             = "";
  }
}

void handle_json_msg(String encoded_json) {
  api_input = encoded_json;
  Serial.print("JSON: ");
  Serial.println(api_input);
}

void handle_rest_msg(String rest_msg) {
  api_input = rest_msg;
  Serial.print("REST: ");
  Serial.println(api_input);
}

void detect_api_type(char inChar) {
  switch (inChar) {
    case '{':
      api_call_type = JSON;
      api_input += inChar;
      break;
    case '/':
      api_call_type = REST;
      break;
    default:
      api_call_type = UNDEFINED;
  }
}
};
#endif
