#pragma once

#include <cstdint>
#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string
#include "comms/config_data/hardware_serial_port.hpp" // for HardwareSerialPort

namespace Cfg {

enum class CommunicationProtocol : uint32_t {
    UnsetCommunicationProtocol,
    SPI,
    I2C,
    HARDWARE_SERIAL,
    DIGITAL_PIN,
};

enum class SensorName : uint32_t {
    UnsetSensorName,
    YawBuffEncoder,
    PitchBuffEncoder,
    FeederBuffEncoder,
    YawIcmImu,
    LeftD200Lidar,
    RightD200Lidar,
    StereoCameraTrigger,

    SensorNameCount
};

struct BuffEncoder : Comms::CommsData {
    uint32_t spi_cs;
    uint32_t spi_miso;
    uint32_t spi_mosi;
    uint32_t spi_sck;

    SensorName encoder_name;
};

struct RevEncoder : Comms::CommsData {
    uint32_t digital_pin;
    bool is_relative;
    SensorName encoder_name;
};

enum class LsmImuAccelRange : uint32_t {
    A_2G,
    A_4G,
    A_8G,
    A_16G,
};

enum class LsmImuGyroRange : uint32_t {
    DPS125,
    DPS250,
    DPS500,
    DPS1000,
    DPS2000,
};

struct LsmImu : Comms::CommsData {
    SensorName imu_name;
    LsmImuAccelRange accel_range;
    LsmImuGyroRange gyro_range;
};

enum class ICMImuAccelRange : uint32_t { 
    A_4G,
    A_8G,
    A_16G,
    A_30G,
};

enum class ICMImuGyroRange : uint32_t { 
    DPS500,
    DPS1000,
    DPS2000,
    DPS4000,
};

struct IcmImu : Comms::CommsData {
    CommunicationProtocol communication_protocol;

    uint32_t spi_cs;
    uint32_t spi_miso;
    uint32_t spi_mosi;
    uint32_t spi_sck;
    ICMImuAccelRange accel_range;
    ICMImuGyroRange gyro_range;

    SensorName imu_name;
};

struct D200Lidar : Comms::CommsData {
    HardwareSerialPort hardware_serial_port;
    float x_offset;
    float y_offset;
    float yaw_offset;
    float valid_region_start;
    float valid_region_end;
    float dead_zone_start;
    float dead_zone_end;
    float dead_zone_check_range;
    SensorName lidar_name;
};

struct LimitSwitch : Comms::CommsData {
    uint32_t digital_pin;
    SensorName switch_name;
};

struct StereoCamTrigger : Comms::CommsData{
    uint32_t digital_trigger_pin_1;
    uint32_t digital_trigger_pin_2;
    uint32_t fps;
    uint32_t trigger_pulse_width;
    SensorName camera_trigger_name;
};
}