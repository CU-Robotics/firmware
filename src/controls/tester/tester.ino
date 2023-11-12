#include "rm_CAN.hpp"
#include "dr16.hpp"
#include "controllers.hpp"


Controllers control;

void setup() {
    Serial.begin();
}

void loop() {
    control.update();
}