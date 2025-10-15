#pragma once

#include <cstdint>
#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string

namespace NewConfig {

enum EncoderName {
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
    }
};

enum D200LidarName {
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
    }
};

enum RealsenseCameraName {
    UnsetRealsenseName,
    Realsense,
};

struct RealsenseCamera : Comms::CommsData {
    float length_of_barrel_from_pitch_axis;
    float height_of_camera_above_barrel;
    float height_of_pitch_axis_above_ground;
    uint32_t camera_name;

    RealsenseCamera() : Comms::CommsData(Comms::TypeLabel::RealsenseCameraConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(RealsenseCamera)) {
        length_of_barrel_from_pitch_axis = 0;
        height_of_camera_above_barrel = 0;
        height_of_pitch_axis_above_ground = 0;
        camera_name = UnsetRealsenseName;
    }
};

}