#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

#include <stdint.h>                             // uintN_t

/// @brief Structure for the ICM sensor.
struct ICMSensorData : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the ICMSensorData struct.
    ICMSensorData() : CommsData(Comms::TypeLabel::ICMSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(ICMSensorData)) { }
    /// @brief Constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the ICMSensorData struct, and also sets the imu_name field.
    /// @param name The name of the sensor
    ICMSensorData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::ICMSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(ICMSensorData)), imu_name(name) { }
    /// @brief The name of the IMU sensor that this data is from.
    Cfg::SensorName imu_name = Cfg::SensorName::UnsetSensorName;
    /// @brief Acceleration in X-axis.
    float accel_X = 0.0;
    /// @brief Acceleration in Y-axis.
    float accel_Y = 0.0;
    /// @brief Acceleration in Z-axis.
    float accel_Z = 0.0;
    /// @brief Gyroscope reading in X-axis.
    float gyro_X = 0.0;
    /// @brief Gyroscope reading in Y-axis.
    float gyro_Y = 0.0;
    /// @brief Gyroscope reading in Z-axis.
    float gyro_Z = 0.0;
    /// @brief Temperature reading.
    float temperature = 0.0;

    /// @brief Print the ICM sensor data for debugging purposes.
    void print() const {
        printf("ICMSensorData - imu_name: %lu, accel_X: %f, accel_Y: %f, accel_Z: %f, gyro_X: %f, gyro_Y: %f, gyro_Z: %f, temperature: %f\n", 
            static_cast<uint32_t>(imu_name), accel_X, accel_Y, accel_Z, gyro_X, gyro_Y, gyro_Z, temperature);
    }
};