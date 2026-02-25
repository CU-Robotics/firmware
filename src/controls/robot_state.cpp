#include "robot_state.hpp"
#include <cstring>

RobotState::RobotState(const std::vector<NewConfig::State>& _state_configurations) {
    for (const auto& state : _state_configurations) {
        robot_state.emplace(state.name, RawState(state));
    }
}

std::expected<void, RobotState::StateError> RobotState::set_state(const RawState& new_state) {
    if(!robot_state.contains(new_state.config.name)){
        Serial.printf("RobotState: StateName %u not found in state config, failed to set value\n", static_cast<uint32_t>(new_state.config.name));
        return std::unexpected(StateError::InvalidStateName);
    }

    robot_state[new_state.config.name].set_values(new_state.values);
    return {};
}

std::expected<const RobotState::RawState&, RobotState::StateError> RobotState::get_state(NewConfig::StateName state_name) const {
    if(!robot_state.contains(state_name)){
        Serial.printf("RobotState: StateName %u not found in state config, failed to get value\n", static_cast<uint32_t>(state_name));
        return std::unexpected(StateError::InvalidStateName);
    }

    return robot_state.at(state_name);
}

std::expected<void, RobotState::StateError> RobotState::set_value(float value, NewConfig::StateName state_name, NewConfig::StateOrder state_order) {
    if(!robot_state.contains(state_name)){
        Serial.printf("RobotState: StateName %u not found in state config, failed to set value\n", static_cast<uint32_t>(state_name));
        return std::unexpected(StateError::InvalidStateName);
    }

    size_t order_index = order_enum_to_array_index(state_order);

    if(order_index >= STATE_ORDER) {
        Serial.printf("RobotState: Order index %u, grabbed from StateOrder %u is out of bounds, failed to set value\n", order_index, static_cast<uint32_t>(state_order));
        return std::unexpected(StateError::OrderIndexOutOfBounds);
    }

    robot_state[state_name].values[order_index] = value;
    return {};
}

std::expected<float, RobotState::StateError> RobotState::get_value(NewConfig::StateName state_name, NewConfig::StateOrder state_order) const {
    if(!robot_state.contains(state_name)){
        Serial.printf("RobotState: StateName %u not found in state config, failed to get value\n", static_cast<uint32_t>(state_name));
        return std::unexpected(StateError::InvalidStateName);
    }

    size_t order_index = order_enum_to_array_index(state_order);

    if(order_index >= STATE_ORDER) {
        Serial.printf("RobotState: Order index %u, grabbed from StateOrder %u is out of bounds, failed to get value\n", order_index, static_cast<uint32_t>(state_order));
        return std::unexpected(StateError::OrderIndexOutOfBounds);
    }

    return robot_state.at(state_name).values[order_index];
}

std::expected<void, RobotState::StateError> RobotState::to_comms_array(float state_data[NUM_STATES][STATE_ORDER]) const {
    for (const auto& [state_name, raw_state] : robot_state) {
        uint32_t index = raw_state.config.array_index;
        if(index >= NUM_STATES) {
            Serial.printf("RobotState: Array index %u, grabbed from state config for state name %u is out of bounds, failed to fill comms data\n", index, static_cast<uint32_t>(state_name));
            return std::unexpected(StateError::StateIndexOutOfBounds);
        }
        std::memcpy(state_data[index], raw_state.values, sizeof(float) * STATE_ORDER);
    }
    return {};
}

std::expected<void, RobotState::StateError> RobotState::from_comms_array(const float state_data[NUM_STATES][STATE_ORDER]) {
    for (auto& [state_name, raw_state] : robot_state) {
        uint32_t index = raw_state.config.array_index;
        if(index >= NUM_STATES) {
            Serial.printf("RobotState: Array index %u, grabbed from state config for state name %u is out of bounds, failed to fill from comms data\n", index, static_cast<uint32_t>(state_name));
            return std::unexpected(StateError::StateIndexOutOfBounds);
        }
        std::memcpy(raw_state.values, state_data[index], sizeof(float) * STATE_ORDER);
    }
    return {};
}