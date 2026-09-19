#pragma once
#include "controls/state.hpp"
#include "utils/safety.hpp"
#include "state.hpp"
#include <cstddef>
#include <array>
#include <vector>
#include <Arduino.h>
#include <array>
#include <optional>

constexpr size_t NUM_STATES = static_cast<size_t>(Cfg::StateName::StateNameCount);

// Forward declaration
namespace Comms {
    template<typename T>
    struct Sendable; // Forward declaration of Sendable
}
/// @brief Manage all of the configured states
class RobotStateArray {
public:
    /// @brief Construct a new RobotStateMap object with the given state configurations. The state configurations are used to set up the state array with the correct state names and limits.
    /// @param _state_configurations List of the state configurations
    RobotStateArray(const std::vector<Cfg::State>& _state_configurations);
    /// @brief Get a mutable reference corresponding to the given state name. Will trigger a safety procedure if the state name is not found in the array.
    /// @param state_name The name of the state to get.
    /// @return A mutable reference to the state object that corresponds to the given state name.
    State &operator[](Cfg::StateName state_name);
    /// @brief Check if a state has been configured and initialized in this array.
    bool has_state(Cfg::StateName state_name) const {
        const size_t idx = static_cast<size_t>(state_name);
        return idx < NUM_STATES && robot_states[idx].has_value();
    }

    /// @brief Insert or reinitialize an individual state directly.
    void set_state(const Cfg::State& state_config) {
        const size_t idx = static_cast<size_t>(state_config.name);
        safety::assert_or_safety_procedure(idx < NUM_STATES, "RobotStateArray: Invalid state index %u", static_cast<uint32_t>(idx));
        robot_states[idx].emplace(state_config);
    }
    /// @brief Get a const reference corresponding to the given state name. Will trigger a safety procedure if the state name is not found in the array.
    /// @param state_name The name of the state to get.
    /// @return A const reference to the state object that corresponds to the given state name.
    const State& operator[](Cfg::StateName state_name) const;
	std::array<std::optional<State>, NUM_STATES>& get_state_array();
    const std::array<std::optional<State>, NUM_STATES>& get_state_array() const;
    /// @brief Send the current state array to comms. 
    // This will convert the state array to a format that can be sent to comms and then send it.
    // The tempalte paramater T is the type of state array to send (eg. target reference state, estimated state) and is used to determine the comms packet format to send.
    template<typename T>
    void send_to_comms() const {
        Comms::Sendable<T> sendable;
        for (size_t i = 0; i < NUM_STATES; i++) {
            if (robot_states[i].has_value()) {
                sendable.data.state[i] = robot_states[i]->get_raw();
            }
        }
        sendable.data.time = millis();
        sendable.send_to_comms();
    }

    /// @brief Copies current raw state values into a C-array for comms.
    /// @param states the robot state array that will be copied from
    void fill_state_array(State::Raw states[NUM_STATES]) const {
        for (size_t i = 0; i < NUM_STATES; i++) {
            if (robot_states[i].has_value()) {
                states[i] = robot_states[i]->get_raw();
            }
        }
    }

    /// @brief Print the state array to the serial monitor
    void print() const;
    /// @brief Update the state array from a comms packet. This will convert the comms packet to the state array format and then update the state array values
    /// @param robot_state_array The array of raw state values received from comms, indexed by the StateName enum values.
    void from_comms_packet(const State::Raw incoming_states[NUM_STATES]);

private:
    /// @brief Array of state names to their corresponding state objects
	std::array<std::optional<State>, NUM_STATES> robot_states;
};
