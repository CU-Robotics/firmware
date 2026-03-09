#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

/// @brief Structure for the buff encoder sensor.
struct BuffEncoderData : Comms::CommsData {
    BuffEncoderData() : CommsData(Comms::TypeLabel::BuffEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(BuffEncoderData)) { }
    BuffEncoderData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::BuffEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(BuffEncoderData)), encoder_name(name) { }
    /// Sensor ID.
    Cfg::SensorName encoder_name;
    /// Measured angle.
    float m_angle;

    void print() const {
        printf("BuffEncoderData - encoder_name: %lu, m_angle: %f\n", static_cast<uint32_t>(encoder_name), m_angle);
    }
};