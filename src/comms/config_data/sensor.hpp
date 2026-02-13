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

enum class HardwareSerialPorts : uint32_t{
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
    HardwareSerialPorts hardware_serial_port;
    uint32_t digital_pin;
}

enum class EncoderName : uint32_t {
    UnsetEncoderName,
    YawEncoder,
    PitchEncoder,
    FeederEncoder,
};

struct BuffEncoder : Comms::CommsData {
    uint32_t chip_select_pin;
    float encoder_offset;
    int feeder_direction;
    float feeder_ratio;
    uint32_t encoder_name;

    CommunicationProtocol communication_protocol;
    Pins pins;

    BuffEncoder() : Comms::CommsData(Comms::TypeLabel::BuffEncoderConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(BuffEncoder)) {
        chip_select_pin = 0;
        encoder_offset = 0;
        feeder_direction = 1;
        feeder_ratio = 1;
        encoder_name = UnsetEncoderName;
    }
};

enum ImuName {
    UnsetImuName,
    YawImu,
};

struct IcmImu : Comms::CommsData {
    float pitch_angle_at_yaw_calibration;
    float yaw_start_angle;
    float pitch_start_angle;
    float roll_start_angle;
    float yaw_axis_vector[3];
    float pitch_axis_vector[3];
    uint32_t imu_name;

    CommunicationProtocol communication_protocol;
    Pins pins;

    IcmImu() : Comms::CommsData(Comms::TypeLabel::IcmImuConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(IcmImu)) {
        pitch_angle_at_yaw_calibration = 0;
        yaw_start_angle = 0;
        pitch_start_angle = 0;
        roll_start_angle = 0;
        imu_name = UnsetImuName;
        for (int i = 0; i < 3; i++) {
            yaw_axis_vector[i] = 0;
            pitch_axis_vector[i] = 0;
        }

        communication_protocol = UnsetCommunicationProtocol;
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
    uint32_t lidar_name;

    CommunicationProtocol communication_protocol;
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
        lidar_name = UnsetLidarName;

        CommunicationProtocol communication_protocol = UnsetCommunicationProtocol;
        Pins pins = {};
    }
};
}