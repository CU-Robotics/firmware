#pragma once

#include <cstdint>
#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string

namespace Cfg {

enum class CommunicationProtocol : uint32_t {
    UnsetCommunicationProtocol,
    SPI,
    I2C,
    HARDWARE_SERIAL,
    DIGITAL_PIN,
};

enum class HardwareSerialPort : uint32_t{
    UnsetHardwareSerialPort,
    Serial1,
    Serial2,
    Serial3,
    Serial4,
    Serial5,
    Serial6,
    Serial7,
    Serial8,
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

    float encoder_offset;
    int feeder_direction;
    float feeder_ratio;
    SensorName encoder_name;

    BuffEncoder() : Comms::CommsData(Comms::TypeLabel::BuffEncoderConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(BuffEncoder)) {
        encoder_offset = 0;
        feeder_direction = 1;
        feeder_ratio = 1;
        encoder_name = SensorName::UnsetSensorName;
    }
};

struct RevEncoder : Comms::CommsData {

};

struct ACS712 : Comms::CommsData {

};

struct LsmImu : Comms::CommsData {
    
};

enum class ICMImuGyroRateRange : uint32_t { 
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
    ICMImuGyroRateRange gyro_rate_range;

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

    CommunicationProtocol communication_protocol;
};

struct StereoCamTrigger : Comms::CommsData{
    uint32_t digital_trigger_pin;
    uint32_t fps;
    uint32_t trigger_pulse_width;
    SensorName trigger_name;
};
}