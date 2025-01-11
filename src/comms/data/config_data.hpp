#pragma once

#include <vector>

#include "comms_data.hpp"

namespace Comms {

struct ConfigData {
    enum class RobotType {
        NONE = 0,
        Hero = 1,
        Engineer = 2,
        Standard1 = 3,
        Standard2 = 4,
        Standard3 = 5,
        Arial = 6,
        Sentry = 7,
        Dart = 8,
        Radar = 9,
        Outpost = 10,
        Base = 11
    } robot_id = RobotType::NONE;

    struct ControllerInfo {
        int controller_type = 0;

        struct XDrivePositionController {
            int full_state_position = 0;
            int full_state_velocity = 0;
            int full_state_position_chassis_heading = 0;
            int full_state_velocity_chassis_heading = 0;
            int low_level_velocity = 0;
            int power_buffer_threshold = 0;
            int power_buffer_critical_threshold = 0;
        } x_drive_position_controller_gains;

        struct XDriveVelocityController {
            int high_level_velocity = 0;
            int full_state_position_chassis_heading = 0;
            int full_state_velocity_chassis_heading = 0;
            int low_level_velocity = 0;
            int power_buffer_threshold = 0;
            int power_buffer_critical_threshold = 0;
        } x_drive_velocity_controller;

        struct YawController {
            int full_state_position = 0;
            int full_state_velocity = 0;
        } yaw_controller_gains;

        struct PitchController {
            int full_state_position = 0;
            int full_state_velocity = 0;
        } pitch_controller_gains;

        struct FlywheelController {
            int full_state_position = 0;
            int full_velocity_position = 0;
        } flywheel_controller_gains;

        struct FeederController {
            int high_level_velocity = 0;
            int low_level_velocity = 0;
        } feeder_controller_gains;

        struct SwitcherController {
            int full_state_position = 0;
            int full_state_velocity = 0;
            int MAGIC_FIRMWARE_NUMBER = 0;
        } swithcer_controller_gains;

    } controller_info;

    struct Gains {
        // ???
    } gains;

    struct GearRatios {
        // ???
    } gear_ratios;

    struct BuffEncoderInfo {
        int chip_select_pin = 0;
        float encoder_offset = 0;
    };

    std::vector<BuffEncoderInfo> buff_encoders;

    struct RevEncoderInfo {
        float distance_from_center = 0;
        float wheel_radius = 0;
    };

};

} // namespace Comms