#pragma once

namespace NewConfig {

enum class StateName : uint32_t {
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

struct ReferenceLimit {
    float min;
    float max;
};

enum class GovernorType : uint32_t {
    None,
    Position,
    Velocity,
    Acceleration,
};

struct State : Comms::CommsData {
    ReferenceLimit reference_limit;
    GovernorType governor_type;
    StateName name;
    uint32_t array_index;
    uint32_t is_wrapping;
};

};