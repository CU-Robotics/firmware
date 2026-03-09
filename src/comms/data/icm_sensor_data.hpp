#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

#include <stdint.h>                             // uintN_t

/// @brief Structure for the ICM sensor.
struct ICMSensorData : Comms::CommsData {
    ICMSensorData() : CommsData(Comms::TypeLabel::ICMSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(ICMSensorData)) { }
    ICMSensorData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::ICMSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(ICMSensorData)), imu_name(name) { }
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

    void print() const {
        printf("ICMSensorData - imu_name: %lu, accel_X: %f, accel_Y: %f, accel_Z: %f, gyro_X: %f, gyro_Y: %f, gyro_Z: %f, temperature: %f\n", 
            static_cast<uint32_t>(imu_name), accel_X, accel_Y, accel_Z, gyro_X, gyro_Y, gyro_Z, temperature);
    }
};