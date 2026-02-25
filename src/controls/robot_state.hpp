#pragma once
constexpr size_t NUM_STATES = 24;
constexpr size_t STATE_ORDER = 3; // position, velocity, acceleration

class RobotState {
public:
    enum class StateError {
        InvalidStateName,
        StateIndexOutOfBounds,
        OrderIndexOutOfBounds, //compile time checks should prevent this error from ever being returned, but it's here for completeness
    };

    struct RawState {
        float values[STATE_ORDER] = {0.0};
        const NewConfig::State& config;

        RawState(const NewConfig::State& _config) : config(_config) {}

        void set_values(float new_values[STATE_ORDER]) {
            std::memcpy(values, new_values, sizeof(values));
        }
    }; 

    RobotState(const std::vector<NewConfig::State>& _state_configurations);

    std::expected<void, RobotState::StateError> set_state(const RawState& new_state);
    std::expected<const RobotState::RawState&, RobotState::StateError> get_state(NewConfig::StateName state_name) const;

    std::expected<void, RobotState::StateError> set_value(float value, NewConfig::StateName state_name, NewConfig::StateOrder state_order);
    std::expected<float, RobotState::StateError> get_value(NewConfig::StateName state_name, NewConfig::StateOrder state_order) const;

    std::expected<void, RobotState::StateError> to_comms_array(float state_data[NUM_STATES][STATE_ORDER]) const;
    std::expected<void, RobotState::StateError> from_comms_array(const float state_data[NUM_STATES][STATE_ORDER]);
    
private:
    std::map<NewConfig::StateName, RawState> robot_state;

private:
    static size_t order_enum_to_array_index(NewConfig::StateOrder state_order) {

        static_assert(static_cast<size_t>(NewConfig::StateOrder::Position)     < STATE_ORDER, "StateOrder enum values must be less than STATE_ORDER");
        static_assert(static_cast<size_t>(NewConfig::StateOrder::Velocity)     < STATE_ORDER, "StateOrder enum values must be less than STATE_ORDER");
        static_assert(static_cast<size_t>(NewConfig::StateOrder::Acceleration) < STATE_ORDER, "StateOrder enum values must be less than STATE_ORDER");

        switch (state_order) {
            case NewConfig::StateOrder::Position:      return static_cast<size_t>(NewConfig::StateOrder::Position);
            case NewConfig::StateOrder::Velocity:      return static_cast<size_t>(NewConfig::StateOrder::Velocity);
            case NewConfig::StateOrder::Acceleration:  return static_cast<size_t>(NewConfig::StateOrder::Acceleration);
        }
        // compile will fail with a warning if a case isn't handled, so this is unreachable
        std::unreachable();
    }
};