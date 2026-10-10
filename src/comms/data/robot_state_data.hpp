#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "controls/state.hpp"
#include "controls/robot_state_array.hpp"

/// @brief Comms data struct for sending the target reference state. 
struct TargetState : Comms::CommsData {
    /// @brief default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the TargetState struct.
    TargetState() : CommsData(Comms::TypeLabel::TargetState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TargetState)) {}
    /// @brief The time at which the target state was generated
    double time = 0.0;
    /// @brief The array of raw state values for each state, indexed by the StateName enum values.
    State::Raw state[static_cast<size_t>(Cfg::StateName::StateNameCount)] = { {0, 0, 0} };
};

/// @brief Comms data struct for sending the reference state output by the reference governor. 
struct ReferenceState : Comms::CommsData {
    /// @brief default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the ReferenceState struct.
    ReferenceState() : CommsData(Comms::TypeLabel::ReferenceState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(ReferenceState)) {}
    /// @brief The time at which the reference state was generated
    double time = 0.0;
    /// @brief The array of raw state values for each state, indexed by the StateName enum values.
    State::Raw state[static_cast<size_t>(Cfg::StateName::StateNameCount)] = { {0, 0, 0} };
};

/// @brief Comms data struct for sending the estimated state.
struct EstimatedState : Comms::CommsData {
    /// @brief default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the EstimatedState struct.
    EstimatedState() : CommsData(Comms::TypeLabel::EstimatedState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(EstimatedState)) {}
    /// @brief The time at which the estimated state was generated
    double time = 0.0;
    /// @brief The array of raw state values for each state, indexed by the StateName enum values.
    State::Raw state[static_cast<size_t>(Cfg::StateName::StateNameCount)] = { {0, 0, 0} };
};

/// @brief Comms data struct for sending the override state. This is used to override firmware's estimated state with something from hive.
struct OverrideState : Comms::CommsData {
    /// @brief default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the OverrideState struct.
    OverrideState() : CommsData(Comms::TypeLabel::OverrideState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(OverrideState)) {}
    /// @brief The time at which the override state was generated
    double time = 0.0;
    /// @brief The array of raw state values for each state, indexed by the StateName enum values.
    State::Raw state[static_cast<size_t>(Cfg::StateName::StateNameCount)] = { {0, 0, 0} };
    /// @brief Whether to actively override firmware's estimated state with this incoming state.
    uint64_t active = false;
};

/// @brief Fixed-size snapshot of the targets actually used by this control loop, queued after safety.
struct AppliedControl : Comms::CommsData {
    /// @brief Initializes the authoritative telemetry packet without dynamic allocation.
    AppliedControl() : CommsData(Comms::TypeLabel::AppliedControl, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(AppliedControl)) {}
    /// @brief Firmware boot milliseconds at this loop's common state/encoder capture boundary.
    double time = 0.0;
    /// @brief Full position/velocity/acceleration targets: yaw, pitch, chassis x, y, heading.
    State::Raw target[5] = {};
    /// @brief Raw yaw/pitch encoder radians decoded successfully in this loop, not field-frame truth.
    float raw_encoders[2] = {};
    /// @brief Actual selected target source: 0 manual transmitter, 1 Hive.
    uint8_t hive_mode = 0;
    /// @brief Actual motor permission after the safety evaluation and CAN write/zero boundary.
    uint8_t motors_armed = 0;
    /// @brief Whether this loop passed an override request to the estimator, before clearing it.
    uint8_t state_override = 0;
    /// @brief Bit i is set only when target[i] is configured; absent states are not measurements.
    uint8_t present = 0;
    /// @brief Actual mode-change reset input used by this loop's reference governor.
    uint8_t mode_changed = 0;
    /// @brief Actual motor permission entering this loop, before its safety evaluation.
    uint8_t previous_armed = 0;
    /// @brief Presence bits for raw_encoders: bit 0 yaw, bit 1 pitch.
    uint8_t encoders_present = 0;
    /// @brief Explicit wire padding, always zero.
    uint8_t reserved[5] = {};
};
static_assert(sizeof(AppliedControl) == 96, "AppliedControl wire layout must match Hive");