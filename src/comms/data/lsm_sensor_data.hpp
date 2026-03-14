#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

#include <stdint.h>                             // uintN_t

/// @brief Structure for the LSM sensor.
struct LsmSensorData : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the LsmSensorData struct.
    LsmSensorData() : CommsData(Comms::TypeLabel::LsmSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(LsmSensorData)) { }
    /// @brief Constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the LsmSensorData struct, and also sets the IMU name.
    /// @param name The name of the LSM sensor that this data corresponds to.
    LsmSensorData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::LsmSensorData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(LsmSensorData)), imu_name(name) { }
    /// @brief The name of the LSM sensor that this data corresponds to.
    Cfg::SensorName imu_name;
    /// @brief Acceleration in X-axis.
    float accel_X;
    /// @brief Acceleration in Y-axis.
    float accel_Y;
    /// @brief Acceleration in Z-axis.
    float accel_Z;
    /// @brief Gyroscope reading in X-axis.
    float gyro_X;
    /// @brief Gyroscope reading in Y-axis.
    float gyro_Y;
    /// @brief Gyroscope reading in Z-axis.
    float gyro_Z;
    /// @brief Temperature reading.
    float temperature;

    /// @brief Print the LSM sensor data to the serial console for debugging purposes.
    void print() const {
        printf("LsmSensorData - imu_name: %lu, accel_X: %f, accel_Y: %f, accel_Z: %f, gyro_X: %f, gyro_Y: %f, gyro_Z: %f, temperature: %f\n", 
            static_cast<uint32_t>(imu_name), accel_X, accel_Y, accel_Z, gyro_X, gyro_Y, gyro_Z, temperature);
    }
};