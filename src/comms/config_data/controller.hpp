#pragma once

#include <cstdint>
#include <stdint.h>     // for uintN_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string
#include "comms/config_data/motor.hpp" // for Motor
#include "state.hpp" // for StateName
#include "safety.hpp" // for assert_or_safety_mode

constexpr uint32_t MAX_GENERIC_MOTOR_USES_PER_CONTROLLER = 16;
constexpr uint32_t MAX_GENERIC_STATE_USES_PER_CONTROLLER = 16;
constexpr uint32_t MAX_SUB_CONTROLLERS_PER_CONTROLLER = 16;

namespace Cfg {

enum class GenericControllerMotorUse : uint32_t {
    ChassisFrontRight,
    ChassisBackRight,
    ChassisBackLeft,
    ChassisFrontLeft,
    Yaw1,
    Yaw2,
    PitchLeft,
    PitchRight,
    FlywheelLeft,
    FlywheelRight,
    Feeder,
};

enum class GenericControllerStateUse : uint32_t {
    ChassisX,
    ChassisY,
    ChassisHeading,
    GimbalYaw,
    GimbalPitch,
    ShooterBallVelocity,
    FeederBallPosition,
};

enum class ControllerType : uint32_t {
    UnsetControllerType,
    XDriveController,
    YawController,
    PitchController,
    FlywheelController,
    FeederController,
};

enum class SubControllerType : uint32_t {
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
    SubControllerType sub_controller_type; 
};

struct Controller : Comms::CommsData {
    MotorName generic_motor_use_to_names[MAX_GENERIC_MOTOR_USES_PER_CONTROLLER]; // list of motor names that this controller controls
    StateName generic_state_use_to_names[MAX_GENERIC_STATE_USES_PER_CONTROLLER]; // list of state names that this controller uses
    SubController type_to_sub_controller[MAX_SUB_CONTROLLERS_PER_CONTROLLER]; // list of subcontrollers that this controller contains
    GearRatios gear_ratios;
    ControllerType controller_type;

    /// @brief Get the subcontroller of this controller with the given type
    /// @param type The type of the subcontroller, as defined by the SubControllerType enum
    /// @return The subcontroller with the given type, or an empty subcontroller if it was not found
    /// @note Since this function is NOT virtual, struct size stays the same and reinterpret_cast works correctly.
    SubController get_sub_controller_by_type(SubControllerType type) const {
        for (int i = 0; i < (int)MAX_SUB_CONTROLLERS_PER_CONTROLLER; i++) {
            if (type_to_sub_controller[i].sub_controller_type == type) {
                return type_to_sub_controller[i];
            }
        }
        // return empty subcontroller if not found
        SubController empty;
        empty.sub_controller_type = SubControllerType::UnsetSubControllerType;
        return empty;
    }

    /// @brief Get the motor name of this controller with the given generic use
    /// @param motor_use The generic use of the motor, as defined by the GenericControllerMotorUse enum
    /// @return The motor name with the given generic use, or an empty motor name if it was not found
    const MotorName& get_motor_name_by_generic_use(GenericControllerMotorUse motor_use) const {
        int motor_index = static_cast<int>(motor_use);
        safety::assert_or_safety_procedure(motor_index > (int)MAX_GENERIC_MOTOR_USES_PER_CONTROLLER || motor_index < 0, "Generic motor use index out of bounds");
        return generic_motor_use_to_names[motor_index];
    }

    /// @brief Get the state name of this controller with the given generic use
    /// @param state_use The generic use of the state, as defined by the GenericControllerStateUse enum
    /// @return The state name with the given generic use, or an empty state name if it was not found
    const StateName& get_state_name_by_generic_use(GenericControllerStateUse state_use) const {
        int state_index = static_cast<int>(state_use);
        safety::assert_or_safety_procedure(state_index > (int)MAX_GENERIC_STATE_USES_PER_CONTROLLER || state_index < 0, "Generic state use index out of bounds");
        return generic_state_use_to_names[state_index];
    }
};
}