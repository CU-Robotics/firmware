#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

/// @brief Structure for the buff encoder sensor.
struct BuffEncoderData : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the BuffEncoderData struct.
    BuffEncoderData() : CommsData(Comms::TypeLabel::BuffEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(BuffEncoderData)) { }
    /// @brief Constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the BuffEncoderData struct, and also sets the encoder name.
    /// @param name The name of the encoder that this data corresponds to.
    BuffEncoderData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::BuffEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(BuffEncoderData)), encoder_name(name) { }
    /// @brief The name of the encoder that this data corresponds to.
    Cfg::SensorName encoder_name;
    /// @brief Measured angle.
    float m_angle;

    /// @brief print the buff encoder data details
    void print() const {
        Serial.printf("BuffEncoderData - encoder_name: %lu, m_angle: %f\n", static_cast<uint32_t>(encoder_name), m_angle);
    }
};