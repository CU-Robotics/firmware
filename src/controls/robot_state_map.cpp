#include "robot_state_map.hpp"
#include "utils/safety.hpp"
#include <cstddef>
#include <cstring>

RobotStateMap::RobotStateMap(const std::vector<Cfg::State>& _state_configurations) {
    for (const auto& state : _state_configurations) {
        robot_state.emplace(state.name, State(state));
    }
}

State& RobotStateMap::operator[](Cfg::StateName state_name) {
    auto it = robot_state.find(state_name);
    safety::assert_or_safety_procedure(it != robot_state.end(), "RobotStateMap: Requested state name %lu not found in robot state map.", static_cast<uint32_t>(state_name));
    return it->second;
}

const State& RobotStateMap::operator[](Cfg::StateName state_name) const {
    auto it = robot_state.find(state_name);
    safety::assert_or_safety_procedure(it != robot_state.end(), "RobotStateMap: Requested state name %lu not found in robot state map.", static_cast<uint32_t>(state_name));
    return it->second;
}

const std::map<Cfg::StateName, State>& RobotStateMap::get_state_map() const {
    return robot_state;
}

std::map<Cfg::StateName, State>& RobotStateMap::get_state_map() {
    return robot_state;
}

void RobotStateMap::from_comms_packet(State::Raw robot_state_array[NUM_STATES]) {
    for (size_t i = 0; i < NUM_STATES; i++) {
        Cfg::StateName state_name = static_cast<Cfg::StateName>(i);
        auto it = robot_state.find(state_name);
        if (it != robot_state.end()) {
            it->second.set_position(robot_state_array[i].position);
            it->second.set_velocity(robot_state_array[i].velocity);
            it->second.set_acceleration(robot_state_array[i].acceleration);
        }
    }
}