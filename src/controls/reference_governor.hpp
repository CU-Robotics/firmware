#include "utils/timing.hpp"
#include "controls/robot_state_map.hpp"

#ifndef STATE_H
#define STATE_H

/// @brief Use reference limits from config to convert ungoverned reference states to generated governed reference states to be sent to controllers.
class Governor {
private:
    /// @brief The govererned reference state map, updated and returned by the step_reference_map function towards the ungoverned reference map.
    RobotStateMap reference_state_map;

    /// @brief Timer for the reference governor
    Timer governor_timer;

    /// @brief counter so dt isnt big in the first loop
    int count = 0;

public:

    /// @brief Construct the reference governor and get the state configurations to set up the reference state map
    /// @param state_configurations The configuration data for the reference state map
    Governor(std::vector<Cfg::State> state_configurations) : reference_state_map(state_configurations) {}

    /// @brief Should not be used often as it defeats the purpose of the reference governor
    /// @param new_reference State map setting the reference map (should equal the robots current estimate)
    void set_reference_map(const RobotStateMap& new_reference);

    /// @brief Sets the position reference for a given state
    /// @param state_name The name of the state to set the reference for  
    /// @param value The value to set
    /// @note This function should be used sparingly, as setting the reference defeats its purpose.
    void set_position_reference(Cfg::StateName state_name, float value);

    /// @brief Sets the velocity reference for a given state
    /// @param state_name The name of the state to set the reference for
    /// @param value The value to set
    /// @note This function should be used sparingly, as setting the reference defeats its purpose.
    void set_velocity_reference(Cfg::StateName state_name, float value);

    /// @brief Sets the acceleration reference for a given state
    /// @param state_name The name of the state to get the reference for
    /// @param value the value to set
    /// @note This function should be used sparingly, as setting the reference defeats its purpose.
    void set_acceleration_reference(Cfg::StateName state_name, float value);

    /// @brief Gives the instantaneous governed state reference matrix (also known as desired state)
    /// @return the current reference state map
    const RobotStateMap& get_reference_map() const;

    /// @brief Steps the reference map towards the ungoverned reference map based on the reference limits and governor type specified in the configuration for each state.
    /// @param ungoverned_reference_map The map of ungoverned reference state (our goal)
    /// @return The map of governed reference states
    const RobotStateMap& step_reference_map(const RobotStateMap& ungoverned_reference_map);
};

#endif