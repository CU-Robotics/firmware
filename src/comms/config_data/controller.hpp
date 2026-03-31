#pragma once

#include <cstdint>
#include <stdint.h>     // for uintN_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string
#include "comms/config_data/motor.hpp" // for Motor
#include "state.hpp" // for StateName
#include "safety.hpp" // for assert_or_safety_mode

/// These values are arbitrary limits for the size of the arrays in the Controller struct. 
// They should be large enough to accommodate any reasonable number of motors, states, or sub controllers per controller configuration.

/// @brief The max number of generic motor uses per controller. 
constexpr uint32_t MAX_GENERIC_MOTOR_USES_PER_CONTROLLER = 16;
/// @brief The max number of generic state uses per controller.
constexpr uint32_t MAX_GENERIC_STATE_USES_PER_CONTROLLER = 16;
/// @brief The max number of subcontrollers per controller.
constexpr uint32_t MAX_SUB_CONTROLLERS_PER_CONTROLLER = 16;

namespace Cfg {

/// @brief This enum represents the different generic uses for motors that a controller can have. These are used to map the specific motors on the robot to their generic uses in the controller config.
///
/// If multiple of the same controller is needed for some reason then the generic uses can be used to differentiate between the different motors that the controllers are controlling.
/// For example, if two Feeder Controllers are needed then the generic motor uses can be used to differentiate between the feeder motor on the first Feeder Controller and the feeder motor on the second Feeder Controller.
/// You would do this in the yaml by doing something like
/// Controllers:
///   Feeder Controller 1:
///     Motors: {Feeder: "Feeder Motor 1"}
///   Feeder Controller 2:
///     Motors: {Feeder: "Feeder Motor 2"}
/// In this example, both Feeder Controllers are controlling a motor that has the generic use of `Feeder`, but they are differentiated by the specific motor name that they are mapped to in the config.
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


/// @brief This enum represents the different generic uses for states that a controller can have. These are used to map the specific states on the robot to their generic uses in the controller config.
///
/// If multiple of the same controller is needed for some reason then the generic uses can be used to differentiate between the different states that the controllers are using for reference.
/// For example, if two Feeder Controllers are needed then the generic state uses can be used to differentiate between the feeder state on the first Feeder Controller and the feeder state on the second Feeder Controller.
/// You would do this in the yaml by doing something like
/// Controllers:
///   Feeder Controller 1:
///     States Used: {FeederBallPosition: "Feeder 1"}
///   Feeder Controller 2:
///     States Used: {FeederBallPosition: "Feeder 2"}
/// In this example, both Feeder Controllers are using a state that has the generic use of `FeederBallPosition`, but they are differentiated by the specific state name that they are mapped to in the config.
enum class GenericControllerStateUse : uint32_t {
    ChassisX,
    ChassisY,
    ChassisHeading,
    GimbalYaw,
    GimbalPitch,
    ShooterBallVelocity,
    FeederBallPosition,
};
/// @brief This enum represents the different types of controllers that can be configured in the config.
enum class ControllerType : uint32_t {
    UnsetControllerType,
    XDriveController,
    YawController,
    PitchController,
    FlywheelController,
    FeederController,
};
/// @brief This enum represents the different types of subcontrollers that a controller can be configured to have.
enum class SubControllerType : uint32_t {
    UnsetSubControllerType,
    
    XYPositionController,
    XYVelocityController,
    ChassisAngleController,
    ChassisAngularVelocityController,
    PowerBufferController,
    
    HighLevelVelocityController,
    LowLevelVelocityController,
    
    FullStatePositionController,
    FullStateVelocityController,
};
/// @brief Gains for a sub controller.
struct Gains {
    /// @brief Proportional gain.
    float p = 0.0;
    /// @brief Integral gain.
    float i = 0.0;
    /// @brief Derivative gain.
    float d = 0.0;
    /// @brief Feedforward gain.
    float f = 0.0;
    /// @brief Power buffer threshold (Joules) at which the controller will start to limit power to the motors.
    float power_buffer_threshold = 0.0;
    /// @brief Power buffer threshold (Joules) at which the controller will cut power completely to the motors.
    float power_buffer_critical_threshold = 0.0;
};
/// @brief Gear ratios for a controller. These are generally used to convert between motor vaues and state values, or vice versa
struct GearRatios {
    /// @brief the ratio between chassis x velocity in m/s and motor velocity in rad/s for the chassis motors.
    float chassis_x_to_motor_rad = 0.0;
    /// @brief the ratio between chassis y velocity in m/s and motor velocity in rad/s for the chassis motors
    float chassis_y_to_motor_rad = 0.0;
    /// @brief convert a value from chassis angle to motor radians
    float chassis_rad_to_motor_rad = 0.0;
    /// @brief direction of motor 1 either 1 or -1
    int motor1_direction = 0;
    /// @brief direction of motor 2 either 1 or -1
    int motor2_direction = 0;
    /// @brief the ratio between ball velocity in m/s and flywheel velocity in rad/s for the flywheel motors.
    float ball_to_flywheel_rad = 0.0;
    /// @brief direction of the feeder motor either 1 or -1
    int feeder_direction = 0;
};
/// @brief Subcontroller configuration for a controller.
struct SubController {
    /// @brief Gains for the subcontroller. Different subcontrollers will use different subsets of these gains, but all of the gains are included here for simplicity and flexibility of the config.
    Gains gains;
    /// @brief The type of the subcontroller, as defined by the SubControllerType enum
    SubControllerType sub_controller_type = SubControllerType::UnsetSubControllerType; 
};

/// @brief Controller configuration struct. This struct contains all the configuration data for a controller, including the specific motors and states that it uses, as well as the subcontrollers that it uses and their gains.
struct Controller : Comms::CommsData {
    /// @brief The specific motor names that this controller uses, indexed by their generic use as defined by the GenericControllerMotorUse enum
    MotorName generic_motor_use_to_names[MAX_GENERIC_MOTOR_USES_PER_CONTROLLER] = { MotorName::UnsetMotorName }; // list of motor names that this controller controls
    /// @brief The specific state names that this controller uses, indexed by their generic use as defined by the GenericControllerStateUse enum
    StateName generic_state_use_to_names[MAX_GENERIC_STATE_USES_PER_CONTROLLER] = { StateName::UnsetStateName }; // list of state names that this controller uses
    /// @brief The subcontrollers that this controller uses.
    SubController type_to_sub_controller[MAX_SUB_CONTROLLERS_PER_CONTROLLER] = { SubController() }; // list of subcontrollers that this controller contains
    /// @brief Gear ratios for this controller.
    GearRatios gear_ratios;
    /// @brief The type of this controller, as defined by the ControllerType enum
    ControllerType controller_type = ControllerType::UnsetControllerType;

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
        safety::safety_procedure("Subcontroller type %u not found in controller config", static_cast<uint32_t>(type));
    }

    /// @brief Get the motor name of this controller with the given generic use
    /// @param motor_use The generic use of the motor, as defined by the GenericControllerMotorUse enum
    /// @return The motor name with the given generic use, or an empty motor name if it was not found
    const MotorName& get_motor_name_by_generic_use(GenericControllerMotorUse motor_use) const {
        int motor_index = static_cast<int>(motor_use);
        safety::assert_or_safety_procedure(motor_index < (int)MAX_GENERIC_MOTOR_USES_PER_CONTROLLER && motor_index >= 0, "Generic motor use index out of bounds");
        return generic_motor_use_to_names[motor_index];
    }

    /// @brief Get the state name of this controller with the given generic use
    /// @param state_use The generic use of the state, as defined by the GenericControllerStateUse enum
    /// @return The state name with the given generic use, or an empty state name if it was not found
    const StateName& get_state_name_by_generic_use(GenericControllerStateUse state_use) const {
        int state_index = static_cast<int>(state_use);
        safety::assert_or_safety_procedure(state_index < (int)MAX_GENERIC_STATE_USES_PER_CONTROLLER && state_index >= 0, "Generic state use index out of bounds");
        return generic_state_use_to_names[state_index];
    }
};
}