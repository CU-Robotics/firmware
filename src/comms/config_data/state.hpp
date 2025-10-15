#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel,

namespace NewConfig {

enum StateName {
    UnsetStateName,
    ChassisX,
    ChassisY,
    ChassisHeading,
    GimbalYaw,
    GimbalPitch,
    Flywheels,
    FeederBalls,
};

struct ReferenceLimit {
    float min;
    float max;
};

enum GovernorType {
    None,
    Position,
    Velocity,
    Acceleration,
};

struct StateInfo : Comms::CommsData {
    ReferenceLimit reference_limits;
    uint32_t governor_type;
    uint32_t state_name;

    StateInfo() : Comms::CommsData(Comms::TypeLabel::StateConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(StateInfo)) {
        reference_limits.min = 0;
        reference_limits.max = 0;
        governor_type = None;
        state_name = UnsetStateName;
    }
};

}