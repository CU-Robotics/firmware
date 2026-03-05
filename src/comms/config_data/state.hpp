#pragma once
#include <stdint.h>
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel,

namespace Cfg {

enum class StateName : uint32_t {
    Unset,
    ChassisX,
    ChassisY,
    ChassisHeading,
    GimbalYaw,
    GimbalPitch,
    Flywheels,
    Feeder,

    StateNameCount
};

enum class StateOrder : uint32_t {
    Position,
    Velocity,
    Acceleration,
};

struct ReferenceLimit {
    float min;
    float max;
};

struct ReferenceLimits {
    ReferenceLimit position;
    ReferenceLimit velocity;
    ReferenceLimit acceleration;
};

struct State : Comms::CommsData {
    ReferenceLimits reference_limits;
    StateOrder governor_type;
    StateName name;
    uint32_t is_wrapping;
};

};