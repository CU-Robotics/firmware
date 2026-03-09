#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string

constexpr uint32_t MAX_NUM_MOTOR_NAMES = 16;

namespace Cfg {

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

    MotorNameCount
};

enum class MotorControllerType : uint32_t {
    UnsetMotorControllerType,
    C620,
    C610,
    MG,
    GIM,
    SDC104,
};

enum class MotorType : uint32_t {
    UnsetMotorType,
    M3508,
    M2006,
    GIM3505,
    GIM4310,
    GIM6010,
    GIM8108,
};

struct Motor : Comms::CommsData {
    MotorControllerType motor_controller_type;
    uint32_t physical_bus;
    uint32_t physical_id;
    MotorType motor_type;
    MotorName motor_name;
};
    
}