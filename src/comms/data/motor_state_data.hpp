#pragma once
#include "comms/data/comms_data.hpp"
#include "comms/config_data/motor.hpp"

/// @brief Motor state data for comms
struct MotorStateData : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the MotorStateData struct.
    MotorStateData() : CommsData(Comms::TypeLabel::MotorStateData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(MotorStateData)) { }

    /// @brief The name of the motor that this data corresponds to.
    Cfg::MotorName motor_name = Cfg::MotorName::UnsetMotorName;
    /// @copybrief MotorState::torque
    float torque = 0.0;
    /// @brief The last torque command sent to this motor
    float commanded_torque = 0.0;
    /// @copybrief MotorState::speed
    float speed = 0.0;
    /// @copybrief MotorState::position
    uint16_t position = 0;
    /// @copybrief MotorState::temperature
    int16_t temperature = 0;
    /// @brief Print the motor state data to the serial console for debugging purposes.
    void print() const {
        Serial.printf("MotorStateData - motor_name: %lu, torque: %f, commanded_torque: %f, speed: %f, position: %u, temperature: %d\n", static_cast<uint32_t>(motor_name), torque, commanded_torque, speed, position, temperature);
    }
};