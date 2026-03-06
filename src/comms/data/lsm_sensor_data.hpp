#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

#include <stdint.h>                             // uintN_t

/// @brief Structure for the LSM sensor.
struct LsmSensorData : Comms::CommsData {
    LsmSensorData() : CommsData(Comms::TypeLabel::LsmSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(LsmSensorData)) { }
    LsmSensorData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::LsmSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(LsmSensorData)), imu_name(name) { }
    /// Sensor ID.
    Cfg::SensorName imu_name;
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