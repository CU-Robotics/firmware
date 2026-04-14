#include "limit_switch.hpp"
#include <Arduino.h>
#include "comms/data/sendable.hpp"

void LimitSwitch::init() {
    pinMode(config.digital_pin, INPUT);  // Set the pin used to measure the limit switch to be an input
}

void LimitSwitch::read() {
    is_pressed = digitalRead(config.digital_pin);
    comms_data.is_pressed = is_pressed;
}

void LimitSwitch::send_to_comms() const {
    Comms::Sendable<LimitSwitchData> sendable;
        
    sendable.data = comms_data;
    sendable.send_to_comms();
}

