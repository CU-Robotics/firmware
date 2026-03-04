#pragma once
#include "state.hpp"

constexpr size_t NUM_STATES = 24;

class RobotStateMap {
public:
    enum class StateMapError {
        InvalidStateName,
    };

    RobotStateMap(const std::vector<NewConfig::State>& _state_configurations);

    State& operator[](NewConfig::StateName state_name);

    target_state_map[StateName::ChassisX].set_position(chassis_pos_x);

    const State& operator[](NewConfig::StateName state_name) const;

    void get_state_for_comms(State states[NUM_STATES]) const;

private:
    std::map<NewConfig::StateName, State> robot_state;
};