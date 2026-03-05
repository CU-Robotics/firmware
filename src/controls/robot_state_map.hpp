#pragma once
#include "state.hpp"

constexpr size_t NUM_STATES = 24;

class RobotStateMap {
public:
    enum class StateMapError {
        InvalidStateName,
    };

    RobotStateMap(const std::vector<Cfg::State>& _state_configurations);

    State& operator[](Cfg::StateName state_name);

    const State& operator[](Cfg::StateName state_name) const;

    void get_state_for_comms(State states[NUM_STATES]) const;

private:
    std::map<Cfg::StateName, State> robot_state;
};