#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

/// @brief Structure for the TOF (Time-of-Flight) sensor.
struct TOFSensorData : Comms::CommsData {
    TOFSensorData() : CommsData(Comms::TypeLabel::TOFSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(TOFSensorData)) { }
    /// Sensor ID.
    uint8_t id;
    /// Latest distance measurement.
    uint16_t latest_distance;
};