#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

/// @brief Structure for the ICM sensor.
struct ICMSensorData : Comms::CommsData {
    ICMSensorData() : CommsData(Comms::TypeLabel::ICMSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(ICMSensorData)) { }
    /// Sensor ID.
    uint8_t id;
    /// Acceleration in X-axis.
    float accel_X;
    /// Acceleration in Y-axis.
    float accel_Y;
    /// Acceleration in Z-axis.
    float accel_Z;
    /// Gyroscope reading in X-axis.
    float gyro_X;
    /// Gyroscope reading in Y-axis.
    float gyro_Y;
    /// Gyroscope reading in Z-axis.
    float gyro_Z;
    /// Temperature reading.
    float temperature;
};