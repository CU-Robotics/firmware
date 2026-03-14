#pragma once
#include <stdint.h>
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel,

namespace Cfg {
/// @brief StateName is a unique identifier for every possible robot state that we want to estimate and control.
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

/// @brief StateOrder represents the different orders of states that we can control and estimate.
enum class StateOrder : uint32_t {
    Position,
    Velocity,
    Acceleration,
};
/// @brief ReferenceLimit represents the limits for a state.
struct ReferenceLimit {
    /// @brief The minimum value
    float min;
    /// @brief The maximum value
    float max;
};

/// @brief ReferenceLimits represents the limits for all orders of a state. States are governed by these values.
struct ReferenceLimits {
    /// @brief The limits for the position order
    ReferenceLimit position;
    /// @brief The limits for the velocity order
    ReferenceLimit velocity;
    /// @brief The limits for the acceleration order
    ReferenceLimit acceleration;
};
/// @brief The `State` struct represents the configuration for a state, including its reference limits, governor type, whether it is wrapping, and its name.
struct State : Comms::CommsData {
    /// @brief The reference limits for this state. The reference governor uses these values to determine how to limit the reference state that the controllers are trying to achieve.
    ReferenceLimits reference_limits;
    /// @brief The governor type for this state. This determines whether the reference governor limits the position, velocity, or acceleration reference for this state.
    StateOrder governor_type;
    /// @brief whether this state's position value wraps between the position reference limits.
    uint32_t is_wrapping;
    /// @brief Name of this state
    StateName name;
};

};