#include "robot_state.hpp"

float RobotState::operator()(NewConfig::StateName state_name, NewConfig::StateOrder state_order) const {
    return try_get_value(state_name, state_order);
}

float RobotState::try_get_value(NewConfig::StateName state_name, NewConfig::StateOrder state_order) const {
    std::optional<size_t> state_index_opt = state_config.state_enum_to_array_index(state_name);
    std::optional<size_t> order_index_opt = state_config.order_enum_to_array_index(state_order);

    if (!state_index_opt.has_value()) {
        Serial.printf("RobotState: Failed to grab the state index for StateName %u, returning 0\n", static_cast<uint32_t>(state_name));
        return 0;
    }

    if (!order_index_opt.has_value()) {
        Serial.printf("RobotState: Failed to grab the order index for StateOrder %u, returning 0\n", static_cast<uint32_t>(state_order));
        return 0;
    }

    size_t state_index = state_index_opt.value();
    size_t order_index = order_index_opt.value();

    if (state_index >= NUM_STATES) {
        Serial.printf("RobotState: State index %u, grabbed from StateName %u is out of bounds, returning 0\n", state_index, static_cast<uint32_t>(state_name));
        return 0;
    }

    if (order_index >= STATE_ORDER) {
        Serial.printf("RobotState: Order index %u, grabbed from StateOrder %u is out of bounds, returning 0\n", order_index, static_cast<uint32_t>(state_order));
        return 0;
    }

    return state[state_index][order_index];
}