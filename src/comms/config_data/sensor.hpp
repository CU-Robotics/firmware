#pragma once

#include <cstdint>
#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string

namespace NewConfig {

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
}

struct Pins {
    uint32_t spi_cs;
    uint32_t spi_sck;
    uint32_t spi_miso;
    uint32_t spi_mosi;
    HardwareSerialPort hardware_serial_port;
    uint32_t digital_pin;
};

enum class BuffEncoderName : uint32_t {
    UnsetBncoderName,
    YawEncoder,
    PitchEncoder,
    FeederEncoder,
};

struct BuffEncoder : Comms::CommsData {
    Pins pins;

    float angle_offset;
    int feeder_direction;
    float feeder_ratio;
    BuffEncoderName encoder_name;

    BuffEncoder() : Comms::CommsData(Comms::TypeLabel::BuffEncoderConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(BuffEncoder)) {
        pins = {};
        angle_offset = 0;
        feeder_direction = 1;
        feeder_ratio = 1;
        encoder_name = BuffEncoderName::UnsetEncoderName;
    }
};

enum class RevEncoderName : uint32_t {
    UnsetRevEncoderName,
};

struct RevEncoder : Comms::CommsData {
    Pins pins;

    float angle_offset;
    RevEncoderName encoder_name;

    RevEncoder() : Comms::CommsData(Comms::TypeLabel::RevEncoderConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(RevEncoder)) {
        pins = {};
        angle_offset = 0;
        encoder_name = RevEncoderName::UnsetRevEncoderName;
    }
};

enum class ICMImuGyroRange : uint32_t {
    UnsetGyroRange,
    DPS500,
    DPS1000,
    DPS2000,
    DPS4000,
};

enum class ICMImuName : uint32_t{
    UnsetImuName,
    YawImu,
};

struct IcmImu : Comms::CommsData {
    CommunicationProtocol communication_protocol;
    Pins pins;
    
    float pitch_angle_at_yaw_calibration;
    float yaw_start_angle;
    float pitch_start_angle;
    float roll_start_angle;
    float yaw_axis_vector[3];
    float pitch_axis_vector[3];
    ICMImuGyroRange gyro_range;
    ICMImuName imu_name;

    IcmImu() : Comms::CommsData(Comms::TypeLabel::IcmImuConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(IcmImu)) {
        pitch_angle_at_yaw_calibration = 0;
        yaw_start_angle = 0;
        pitch_start_angle = 0;
        roll_start_angle = 0;
        imu_name = ICMImuName::UnsetImuName;
        for (int i = 0; i < 3; i++) {
            yaw_axis_vector[i] = 0;
            pitch_axis_vector[i] = 0;
        }

        gyro_range = ICMImuGyroRange::UnsetGyroRange;
        communication_protocol = CommunicationProtocol::UnsetCommunicationProtocol;
        pins = {};
    }
};

enum class LSMImuName : uint32_t{
    UnsetLSMImuName,
};

struct LSMImu : Comms::CommsData {
    Pins pins;
    LSMImuName imu_name;

    LSMImu() : Comms::CommsData(Comms::TypeLabel::LSMImuConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(LSMImu)) {
        imu_name = LSMImuName::UnsetLSMImuName;
        pins = {};
    }
};

enum class D200LidarName : uint32_t {
    UnsetLidarName,
    LeftLidar,
    RightLidar,
};

struct D200Lidar : Comms::CommsData {
    float x_offset;
    float y_offset;
    float yaw_offset;
    float valid_region_start;
    float valid_region_end;
    float dead_zone_start;
    float dead_zone_end;
    float dead_zone_check_range;
    D200LidarName lidar_name;

    Pins pins;

    D200Lidar() : Comms::CommsData(Comms::TypeLabel::D200LidarConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(D200Lidar)) {
        x_offset = 0;
        y_offset = 0;
        yaw_offset = 0;
        valid_region_start = 0;
        valid_region_end = 0;
        dead_zone_start = 0;
        dead_zone_end = 0;
        dead_zone_check_range = 0;
        lidar_name = D200LidarName::UnsetLidarName;

        pins = {};
    }
};

enum class StereoCameraTriggerName : uint32_t {
    UnsetStereoCameraTriggerName,
    LeftStereoCameraTrigger,
    RightStereoCameraTrigger,
};

struct StereoCamTrigger : Comms::CommsData{
    Pins pins;

    uint32_t fps;
    uint32_t trigger_pulse_width;
    StereoCameraTriggerName trigger_name;

    StereoCamTrigger() : Comms::CommsData(Comms::TypeLabel::StereoCameraTriggerConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(StereoCamTrigger)) {
        pins = {};
        fps = 0;
        trigger_pulse_width = 0;
        trigger_name = StereoCameraTriggerName::UnsetStereoCameraTriggerName;
    }
};
}