#pragma once

namespace NewConfig {

enum class StateName : uint32_t {
    Unset,
    ChassisX,
    ChassisY,
    ChassisHeading,
    GimbalYaw,
    GimbalPitch,
    Flywheels,
    Feeder,
};

enum class StateOrder : uint32_t {
    Position,
    Velocity,
    Acceleration,
};

struct ReferenceLimits {
    ReferenceLimit position;
    ReferenceLimit velocity;
    ReferenceLimit acceleration;
};

struct ReferenceLimit {
    float min;
    float max;
};

struct State : Comms::CommsData {
    ReferenceLimits reference_limits;
    StateOrder governor_type;
    StateName name;
    uint32_t is_wrapping;
};

};