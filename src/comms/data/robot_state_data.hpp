#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "controls/state.hpp"
#include "controls/robot_state_map.hpp"

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