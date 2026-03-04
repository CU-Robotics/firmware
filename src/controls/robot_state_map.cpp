#include "robot_state.hpp"
#include <cstddef>
#include <cstring>

RobotStateMap::RobotStateMap(const std::vector<NewConfig::State>& _state_configurations) {
    for (const auto& state : _state_configurations) {
        robot_state.emplace(state.name, State(state));
    }
}

State& RobotStateMap::operator[](NewConfig::StateName state_name) {

    auto it = robot_state.find(state_name);
    assert(it != robot_state.end());

    return it->second;
}

const State& RobotStateMap::operator[](NewConfig::StateName state_name) const {
    return robot_state.at(state_name);
}

//need some way to specify how many states there are
void RobotStateMap::get_state_for_comms(State states[NUM_STATES]) const {
    
}