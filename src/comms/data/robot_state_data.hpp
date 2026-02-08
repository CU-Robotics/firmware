#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif
// TODO: replace with kyle3's new state struct
/// @brief Structure for the full robot state including time
struct TempRobotState : Comms::CommsData {
    TempRobotState() : CommsData(Comms::TypeLabel::TempRobotState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TempRobotState)) { }
  
    /// @brief Time of the teensy
    double time = 0.0;
    /// @brief Full robot state array
    float state[24][3] = { {0} };
    /// @brief The delay in communication between the teensy and the khadas
    double comms_delay = 0;
};

/// @brief Full robot state, in the form of TargetState
struct TargetState : Comms::CommsData {
    TargetState() : CommsData(Comms::TypeLabel::TargetState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TargetState)) { }

    /// @brief Time of the teensy
    double time = 0.0;
    /// @brief Full robot state array
    float state[24][3] = { {0} };
    /// @brief The delay in communication between the teensy and the khadas
    double comms_delay = 0;
};

/// @brief Full robot state, in the form of EstimatedState
struct EstimatedState : Comms::CommsData {
    EstimatedState() : CommsData(Comms::TypeLabel::EstimatedState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(EstimatedState)) { }
  
    /// @brief Time of the teensy
    double time = 0.0;
    /// @brief Full robot state array
    float state[24][3] = { {0} };
    /// @brief The delay in communication between the teensy and the khadas
    double comms_delay = 0;
};

/// @brief Full robot state, in the form of OverrideState
struct OverrideState : Comms::CommsData {
    OverrideState() : CommsData(Comms::TypeLabel::OverrideState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(OverrideState)) { }
  
    /// @brief Time of the teensy
    double time = 0.0;
    /// @brief Full robot state array
    float state[24][3] = { {0} };
    /// @brief The delay in communication between the teensy and the khadas
    double comms_delay = 0;
    /// @brief Whether this request is active or not
    bool active = false;
};
