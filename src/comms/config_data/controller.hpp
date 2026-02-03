#pragma once

#include <stdint.h>     // for uintN_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string
#include "comms/config_data/motor.hpp" // for Motor

#define CONTROLLER_MOTORS_SIZE 8
#define CONTROLLER_SUB_CONTROLLERS_SIZE 8

namespace NewConfig {

enum ControllerType : uint8_t {
    UnsetControllerType,
    XDrivePositionController,
    XDriveVelocityController,
    YawController,
    PitchController,
    FlywheelController,
    FeederController,
};

enum SubControllerType : uint8_t {
    UnsetSubControllerType,
    
    XYPositionController,
    XYVelocityController,
    ChassisAngleController,
    ChassisAngularVelocityController,
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
    ControllerType controller_type;

    Controller() : Comms::CommsData(Comms::TypeLabel::ControllerConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(Controller)) {
        for (int i = 0; i < CONTROLLER_MOTORS_SIZE; i++) {
            motors[i] = UnsetMotorName;
        }
        for (int i = 0; i < CONTROLLER_SUB_CONTROLLERS_SIZE; i++) {
            sub_controllers[i].sub_controller_type = UnsetSubControllerType;
        }
        controller_type = UnsetControllerType;
    }

    SubController get_sub_controller_by_type(uint32_t type) const {
        for (int i = 0; i < CONTROLLER_SUB_CONTROLLERS_SIZE; i++) {
            if (sub_controllers[i].sub_controller_type == type) {
                return sub_controllers[i];
            }
        }
        // return empty subcontroller if not found
        SubController empty;
        empty.sub_controller_type = UnsetSubControllerType;
        return empty;
    }
};
}