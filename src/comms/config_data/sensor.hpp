#pragma once

#include <cstdint>
#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string
#include "comms/config_data/hardware_serial_port.hpp" // for HardwareSerialPort

namespace Cfg {
/// @brief Different communication protocols that can be used by sensors on the robot.
// This information is neccessary because some sensors can be configured to use different communication protocols.
enum class CommunicationProtocol : uint32_t {
    UnsetCommunicationProtocol,
    SPI,
    I2C,
    HARDWARE_SERIAL,
    DIGITAL_PIN,
};
/// @brief Difference sensor that a robot can have.
enum class SensorName : uint32_t {
    UnsetSensorName,
    YawBuffEncoder,
    PitchBuffEncoder,
    FeederBuffEncoder,
    YawIcmImu,
    LeftD200Lidar,
    RightD200Lidar,
    StereoCameraTrigger,
    LowerFeederEncoder,
    UpperFeederEncoder,

    SensorNameCount
};
/// @brief Buff encoder configuration struct.
struct BuffEncoder : Comms::CommsData {
    // Buffencoder uses SPI
    /// @brief Chip Select pin for SPI
    uint32_t spi_cs;
    /// @brief Master In Slave Out pin for SPI
    uint32_t spi_miso;
    /// @brief Master Out Slave In pin for SPI
    uint32_t spi_mosi;
    /// @brief Serial Clock pin for SPI
    uint32_t spi_sck;
    /// @brief Name of this encoder
    SensorName encoder_name;
};

/// @brief Rev encoder configuration struct.
struct RevEncoder : Comms::CommsData {
    /// @brief Digital pin for the encoder signal
    uint32_t digital_pin;
    /// @brief Whether this encoder is relative (angle 0 is the angle upon first read) or absolute (angle 0 is a fixed angle that does not change based on when the encoder is powered on)
    bool is_relative;
    /// @brief Name of this encoder
    SensorName encoder_name;
};

/// @brief Acceleration range values for LSM IMU.
enum class LsmImuAccelRange : uint32_t {
    A_2G,
    A_4G,
    A_8G,
    A_16G,
};

/// @brief Gyroscope range values for LSM IMU.
enum class LsmImuGyroRange : uint32_t {
    DPS125,
    DPS250,
    DPS500,
    DPS1000,
    DPS2000,
};

/// @brief LSM IMU configuration struct.
struct LsmImu : Comms::CommsData {
    /// @brief Acceleration range for the LSM IMU in Gs
    LsmImuAccelRange accel_range;
    /// @brief Gyroscope range for the LSM IMU in degrees per second
    LsmImuGyroRange gyro_range;
    /// @brief Name of this sensor
    SensorName imu_name;
};

/// @brief Acceleration range values for ICM IMU.
enum class ICMImuAccelRange : uint32_t { 
    A_4G,
    A_8G,
    A_16G,
    A_30G,
};
/// @brief Gyroscope range values for ICM IMU.
enum class ICMImuGyroRange : uint32_t { 
    DPS500,
    DPS1000,
    DPS2000,
    DPS4000,
};
/// @brief ICM IMU configuration struct.
struct IcmImu : Comms::CommsData {
    /// @brief The communication protocol that the ICM IMU is configured to use. 
    // This is necessary because the ICM IMU can be configured to use either SPI or I2C. Any other value will trigger a safety procedure.
    CommunicationProtocol communication_protocol;

    /// SPI pins only used if the communication protocol is set to SPI.

    /// @brief Chip Select pin for SPI
    uint32_t spi_cs;
    /// @brief Master In Slave Out pin for SPI
    uint32_t spi_miso;
    /// @brief Master Out Slave In pin for SPI
    uint32_t spi_mosi;
    /// @brief Serial Clock pin for SPI
    uint32_t spi_sck;
    /// @brief Acceleration range for the ICM IMU in Gs
    ICMImuAccelRange accel_range;
    /// @brief Gyroscope range for the ICM IMU in degrees per second
    ICMImuGyroRange gyro_range;
    /// @brief Name of this sensor
    SensorName imu_name;
};
/// @brief D200 Lidar configuration struct.
struct D200Lidar : Comms::CommsData {
    /// @brief The hardware serial port that this lidar is connected to
    HardwareSerialPort hardware_serial_port;
    /// @brief The x offset of the lidar from the center of the robot in meters. This is used for localization and should be measured accurately.
    float x_offset;
    /// @brief The y offset of the lidar from the center of the robot in meters. This is used for localization and should be measured accurately.
    float y_offset;
    /// @brief The yaw offset of the lidar from the center of the robot in radians. This is used for localization and should be measured accurately.
    float yaw_offset;
    /// @brief The start angle of the valid lidar points in radians
    float valid_region_start;
    /// @brief The end angle of the valid lidar points in radians
    float valid_region_end;
    /// @brief The start angle of the dead zone, in radians.
    float dead_zone_start;
    /// @brief The end angle of the dead zone, in radians.
    float dead_zone_end;
    /// @brief The range within which to check for dead zones, in radians.
    float dead_zone_check_range;
    /// @brief Name of this sensor
    SensorName lidar_name;
};

/// @brief Limit switch configuration struct.
struct LimitSwitch : Comms::CommsData {
    /// @brief The digital pin that this limit switch is connected to
    uint32_t digital_pin;
    /// @brief Name of this sensor
    SensorName switch_name;
};
/// @brief Stereo camera trigger configuration struct.
struct StereoCamTrigger : Comms::CommsData{
    /// @brief The digital pin connected to the frame trigger input on the first stereo camera.
    uint32_t digital_trigger_pin_1;
    /// @brief The digital pin connected to the frame trigger input on the second stereo camera.
    uint32_t digital_trigger_pin_2;
    /// @brief The frames per second that the cameras should be triggered at. This is used to calculate the frequency of the interrupt timers for the camera trigger.
    uint32_t fps;
    /// @brief The width of the trigger pulse, in microseconds. This is used to define the square wave that triggers the camera frame exposures.
    uint32_t trigger_pulse_width;
    /// @brief Name of this sensor
    SensorName camera_trigger_name;
};
}