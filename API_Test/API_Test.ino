#include "API_Library.h"

API_Library api = API_Library();

void setup() {
  // put your setup code here, to run once:
  api.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  api.loop();
}

void serialEvent() {
  api.serialEvent();
}
