#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

/// @brief Structure for the buff encoder sensor.
struct BuffEncoderData : Comms::CommsData {
    BuffEncoderData() : CommsData(Comms::TypeLabel::BuffEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(BuffEncoderData)) { }
    /// Sensor ID.
    Cfg::SensorName encoder_name;
    /// Measured angle.
    float m_angle;
};