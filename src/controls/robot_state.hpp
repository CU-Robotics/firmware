#pragma once
constexpr size_t NUM_STATES = 24;
constexpr size_t STATE_ORDER_ = 3;

class RobotState {
public:
    RobotState(const NewConfig::State& _state_config) : state_config(_state_config) { }

    float operator()(NewConfig::StateName state_name, NewConfig::StateOrder state_order) const;
    
private:
    /// @brief the state of this robot.
    float state[NUM_STATES][STATE_ORDER] = {0};
    
    const NewConfig::State& state_config;
private: 

    float try_get_value(NewConfig::StateName state_name, NewConfig::StateOrder state_order) const;

};