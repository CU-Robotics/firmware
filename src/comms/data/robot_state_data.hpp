#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include "controls/state.hpp"
#include "controls/robot_state_map.hpp"

struct TargetState : Comms::CommsData {
    TargetState() : CommsData(Comms::TypeLabel::TargetState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(TargetState)) {}
    double time = 0.0;
    State::Raw state[NUM_STATES] = { {0, 0, 0} };
};

struct EstimatedState : Comms::CommsData {
    EstimatedState() : CommsData(Comms::TypeLabel::EstimatedState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(EstimatedState)) {}
    double time = 0.0;
    State::Raw state[NUM_STATES] = { {0, 0, 0} };
};

struct OverrideState : Comms::CommsData {
    OverrideState() : CommsData(Comms::TypeLabel::OverrideState, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(OverrideState)) {}
    double time = 0.0;
    State::Raw state[NUM_STATES] = { {0, 0, 0} };
    bool active = false;
};

