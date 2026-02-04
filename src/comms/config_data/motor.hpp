#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel, to_string

namespace NewConfig {

enum MotorName {
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
};

enum MotorControllerType {
    UnsetMotorControllerType,
    C620,
    C610,
    MG,
    GIM,
    SDC104,
};

enum MotorType {
    UnsetMotorType,
    M3508,
    M2006,
    GIM3505,
    GIM4310,
    GIM6010,
    GIM8108,
};

struct Motor : Comms::CommsData {
    uint32_t motor_controller_type;
    uint32_t physical_bus;
    uint32_t physical_id;
    uint32_t motor_type;
    uint32_t motor_name;

    Motor() : Comms::CommsData(Comms::TypeLabel::MotorConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(Motor)) {
        motor_controller_type = UnsetMotorControllerType;
        physical_bus = 0;
        physical_id = 0;
        motor_type = UnsetMotorType;
        motor_name = UnsetMotorName;
    }
};
    
}