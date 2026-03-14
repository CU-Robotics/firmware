#pragma once
#include "controls/state.hpp"
#include "state.hpp"
#include <cstddef>
#include <map>
#include <vector>
#include <Arduino.h>

constexpr size_t NUM_STATES = static_cast<size_t>(Cfg::StateName::StateNameCount);

// Forward declaration
namespace Comms {
    template<typename T>
    class Sendable; // Forward declaration of Sendable
}
/// @brief Manage all of the congiured states
class RobotStateMap {
public:
    /// @brief Construct a new RobotStateMap object with the given state configurations. The state configurations are used to set up the state map with the correct state names and limits.
    /// @param _state_configurations List of the state configurations
    RobotStateMap(const std::vector<Cfg::State>& _state_configurations);
    /// @brief Get a mutable reference corresponding to the given state name. Will trigger a safety procedure if the state name is not found in the map.
    /// @param state_name The name of the state to get.
    /// @return A mutable reference to the state object that corresponds to the given state name.
    State& operator[](Cfg::StateName state_name);
    /// @brief Get a const reference corresponding to the given state name. Will trigger a safety procedure if the state name is not found in the map.
    /// @param state_name The name of the state to get.
    /// @return A const reference to the state object that corresponds to the given state name.
    const State& operator[](Cfg::StateName state_name) const;
    /// @brief Get the entire state map as a mutable reference.
    /// @return std::map of state names to their corresponding state objects, as a mutable reference.
    std::map<Cfg::StateName, State>& get_state_map();
    /// @brief Get the entire state map as a const reference.
    /// @return std::map of state names to their corresponding state objects, as a const reference.
    const std::map<Cfg::StateName, State>& get_state_map() const;

    /// @brief Send the current state map to comms. 
    // This will convert the state map to a format that can be sent to comms and then send it.
    // The tempalte paramater T is the type of state map to send (eg. target reference state, estimated state) and is used to determine the comms packet format to send.
    template<typename T>
    void send_to_comms() const {
        Comms::Sendable<T> sendable;
        for (const auto& [state_name, state] : robot_state) {
            sendable.data.state[static_cast<size_t>(state_name)] = state.get_raw();
        }

        sendable.data.time = millis();
        sendable.send_to_comms();
    }
    /// @brief Print the state map to the serial monitor
    void print();
    /// @brief Update the state map from a comms packet. This will convert the comms packet to the state map format and then update the state map values
    /// @param robot_state_array The array of raw state values received from comms, indexed by the StateName enum values.
    void from_comms_packet(State::Raw robot_state_array[NUM_STATES]);

private:
    /// @brief Map of state names to their corresponding state objects
    std::map<Cfg::StateName, State> robot_state;
};