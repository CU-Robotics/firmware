#pragma once
#include "controls/state.hpp"
#include <map>
#include <vector>
#include <Arduino.h>

// Forward declaration
namespace Comms {
    template<typename T>
    class Sendable; // Forward declaration of Sendable
}

constexpr size_t NUM_STATES = 24;

class RobotStateMap {
public:
    enum class StateMapError {
        InvalidStateName,
    };

    RobotStateMap(const std::vector<Cfg::State>& _state_configurations);

    State& operator[](Cfg::StateName state_name);

    const State& operator[](Cfg::StateName state_name) const;

    const std::map<Cfg::StateName, State>& get_state_map() const;

    std::map<Cfg::StateName, State>& get_state_map();

    template<typename T>
    void send_to_comms() const {
        Comms::Sendable<T> sendable;
        for (const auto& [state_name, state] : robot_state) {
            sendable.data.state[static_cast<size_t>(state_name)] = state.get_raw();
        }

        sendable.data.time = millis();
        sendable.send_to_comms();
    }

    void from_comms_packet(State::Raw robot_state_array[NUM_STATES]);

private:
    std::map<Cfg::StateName, State> robot_state;
};