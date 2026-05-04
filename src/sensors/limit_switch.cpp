#include "limit_switch.hpp"
#include <Arduino.h>
#include "comms/data/sendable.hpp"

void LimitSwitch::init_impl() {
    pinMode(config.digital_pin, INPUT);  // Set the pin used to measure the limit switch to be an input
}

void LimitSwitch::read_impl() {
    is_pressed = digitalRead(config.digital_pin);
    comms_data.is_pressed = is_pressed;
}

void LimitSwitch::send_to_comms_impl() const {
    Comms::Sendable<LimitSwitchData> sendable;
        
    sendable.data = comms_data;
    sendable.send_to_comms();
}
void LimitSwitch::print_live_data_impl() {
    Serial.printf(" [Limit Switch %d] Status: %s\n", 
                  (int)config.switch_name, get_is_pressed() ? "PRESSED (CLOSED)" : "OPEN");
}

