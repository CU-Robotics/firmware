#pragma once

#include <stdint.h>     // for uintN_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string
#include "comms/config_data/motor.hpp" // for Motor

#define CONTROLLER_MOTORS_SIZE 8
#define CONTROLLER_SUB_CONTROLLERS_SIZE 8

namespace NewConfig {

enum ControllerName : uint8_t {
    UnsetControllerName,
    XDrivePositionController,
    XDriveVelocityController,
    YawController,
    PitchController,
    FlywheelController,
    FeederController,
};

enum SubControllerName : uint8_t {
    UnsetSubControllerName,
    
    XYPositionController,
    XYVelocityController,
    ChassisAngleController,
    HeadingVelocityController,
    PowerBufferController,
    
    LowLevelVelocityController,
    HighLevelVelocityController,
    
    FullStatePositionController,
    FullStateVelocityController,
};

struct Gains {
    float p;
    float i;
    float d;
    float f;
    float power_buffer_threshold;
    float power_buffer_critical_threshold;
};

struct GearRatios {
    float chassis_x_to_motor_rad;
    float chassis_y_to_motor_rad;
    float chassis_rad_to_motor_rad;
    int motor1_direction;
    int motor2_direction;
    int ball_to_flywheel_rad;
    int feeder_direction;
};

struct SubController {
    Gains gains;
    uint32_t sub_controller_type; 
};

struct Controller : Comms::CommsData {
    uint32_t motors[CONTROLLER_MOTORS_SIZE];  // list of motor ids that this controller controls
    SubController sub_controllers[CONTROLLER_SUB_CONTROLLERS_SIZE]; 
    GearRatios gear_ratios;
    uint32_t controller_type; 

    Controller() : Comms::CommsData(Comms::TypeLabel::ControllerConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(Controller)) {
        for (int i = 0; i < CONTROLLER_MOTORS_SIZE; i++) {
            motors[i] = UnsetMotorName;
        }
        for (int i = 0; i < CONTROLLER_SUB_CONTROLLERS_SIZE; i++) {
            sub_controllers[i].sub_controller_type = UnsetSubControllerName;
        }
        controller_type = UnsetControllerName;
    }
};

}