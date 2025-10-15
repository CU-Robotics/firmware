#include "Magnet.hpp"
#include <Arduino.h>

Magnet::Magnet() {
    pinMode(MAGNET_PIN, OUTPUT);
    state = false;
}

void Magnet::on() {
    digitalWrite(MAGNET_PIN, HIGH);
    state = true;
}

void Magnet::off() {
    digitalWrite(MAGNET_PIN, LOW);
    state = false;
}

bool Magnet::get_state() {
    return state;
}