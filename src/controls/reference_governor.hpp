#pragma once
#include "utils/timing.hpp"
#include "controls/robot_state_array.hpp"



/// @brief Use reference limits from config to convert ungoverned reference states to generated governed reference states to be sent to controllers.
class Governor {
private:
    /// @brief The govererned reference state array, updated and returned by the step_reference_array function towards the ungoverned reference array.
    RobotStateArray reference_state_array;

    /// @brief Timer for the reference governor
    Timer governor_timer;

    /// @brief counter so dt isnt big in the first loop
    int count = 0;

public:

    /// @brief Construct the reference governor and get the state configurations to set up the reference state array
    /// @param state_configurations The configuration data for the reference state array
    Governor(std::vector<Cfg::State> state_configurations) : reference_state_array(state_configurations) {}

    /// @brief Set the governed reference array.
    /// @note Should not be used often as it defeats the purpose of the reference governor
    /// @param new_reference State array setting the reference array (should equal the robots current estimate)
    void set_reference_array(const RobotStateArray& new_reference);

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
    /// @return the current reference state array
    const RobotStateArray& get_reference_array() const;

    /// @brief Steps the reference array towards the ungoverned reference array based on the reference limits and governor type specified in the configuration for each state.
    /// @param ungoverned_reference_array The array of ungoverned reference state (our goal)
    /// @return The array of governed reference states
    const RobotStateArray& step_reference_array(const RobotStateArray& ungoverned_reference_array);
};
