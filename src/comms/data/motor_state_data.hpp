#pragma once
#include "comms/data/comms_data.hpp"
#include "comms/config_data/motor.hpp"

struct MotorStateData : Comms::CommsData {
    MotorStateData() : CommsData(Comms::TypeLabel::MotorStateData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(MotorStateData)) { }

    Cfg::MotorName motor_name;
    float torque;
    float speed;
    uint16_t position;
    int16_t temperature;

    void print() const {
        Serial.printf("MotorStateData - motor_name: %lu, torque: %f, speed: %f, position: %u, temperature: %d\n", static_cast<uint32_t>(motor_name), torque, speed, position, temperature);
    }
};