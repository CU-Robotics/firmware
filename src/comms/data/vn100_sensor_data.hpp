#pragma once

#include "comms/data/comms_data.hpp"
#include "comms/config_data/sensor.hpp"

/// @brief VN100 samples in native sensor units; layout matches Hive's Vn100SensorData.
/// @note Validity indicates receipt, not freshness. Compare each startup timestamp.
struct Vn100SensorData : Comms::CommsData {
    /// @brief Create an Ethernet telemetry record using the standard IMU priority.
    explicit Vn100SensorData(Cfg::SensorName name = Cfg::SensorName::UnsetSensorName)
        : CommsData(Comms::TypeLabel::Vn100SensorData, Comms::PhysicalMedium::Ethernet,
                    Comms::Priority::Medium, sizeof(Vn100SensorData)), imu_name(name) {}
    /// @brief Sensor identity.
    Cfg::SensorName imu_name;
    /// @brief Bit 0: IMU received; bit 1: attitude received. Other bits reserved.
    uint32_t valid = 0;
    /// @brief IMU sample time in nanoseconds since sensor startup.
    uint64_t imu_time = 0;
    /// @brief Attitude sample time in nanoseconds since sensor startup.
    uint64_t attitude_time = 0;
    /// @brief Body acceleration including gravity, in m/s^2 (X, Y, Z).
    float accel[3] = {};
    /// @brief Body angular velocity in rad/s (X, Y, Z).
    float gyro[3] = {};
    /// @brief Temperature in degrees Celsius.
    float temperature = 0;
    /// @brief Pressure in kPa.
    float pressure = 0;
    /// @brief Body magnetic field in Gauss (X, Y, Z).
    float mag[3] = {};
    /// @brief Yaw, pitch and roll in degrees.
    float ypr[3] = {};
    /// @brief Quaternion (X, Y, Z, scalar).
    float quaternion[4] = {};
    /// @brief Linear acceleration in the NED frame, in m/s^2.
    float lin_accel_ned[3] = {};
    /// @brief VN100 SensSat bits 0..9 (mag XYZ, accel XYZ, gyro XYZ, pressure).
    uint32_t saturation = 0;
};
static_assert(sizeof(Vn100SensorData) == 120, "VN100 telemetry must match Hive");
