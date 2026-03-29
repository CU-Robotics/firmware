#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string

namespace Cfg {
/// @brief Different motors a robot can have
enum class MotorName : uint32_t{
    UnsetMotorName,
    Chassis1,
    Chassis2,
    Chassis3,
    Chassis4,
    Yaw1,
    Yaw2,
    Pitch1,
    Pitch2,
    Flywheel1,
    Flywheel2,
    Feeder,
    UpperFeeder,
    LowerFeederClose,
    LowerFeederFar,

    MotorNameCount
};
/// @brief Different types of motor controllers that motors can use
enum class MotorControllerType : uint32_t {
    UnsetMotorControllerType,
    C620,
    C610,
    MG,
    GIM,
    SDC104,
};
/// @brief Different types of motors that can be used on the robot.
enum class MotorType : uint32_t {
    UnsetMotorType,
    M3508,
    M2006,
    GIM3505,
    GIM4310,
    GIM6010,
    GIM8108,
};
/// @brief The `Motor` struct represents the configuration for a motor, including its controller type, physical bus and id, motor type, and motor name.
struct Motor : Comms::CommsData {
    /// @brief The motor controller that this motor uses.
    MotorControllerType motor_controller_type = MotorControllerType::UnsetMotorControllerType;
    /// @brief Which physical bus this motor is on.
    uint32_t physical_bus = 0;
    /// @brief The id of this motor on its physical bus.
    uint32_t physical_id = 0;
    /// @brief The type of this motor
    MotorType motor_type = MotorType::UnsetMotorType;
    /// @brief The name of this motor.
    MotorName motor_name = MotorName::UnsetMotorName;
};
    
}