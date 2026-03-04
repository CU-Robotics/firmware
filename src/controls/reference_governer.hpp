#include "../utils/timing.hpp"
#include "robot_state.hpp"

#ifndef STATE_H
#define STATE_H

/// @brief Use reference limits from config to convert ungoverned reference states to generated governed reference states to be sent to controllers.
class Governor {
private:
    // This is a sample state (it does not represent every robot):
    // {x, y, psi (chassis angle), theta (yaw angle), phi (pitch angle), feed, flywheel}
    // In this example case, as with all other cases, the unused state rows are kept blank.

    RobotState reference_state;

    /// @brief Timer for the reference governor
    Timer governor_timer;

    /// @brief counter so dt isnt big in the first loop
    int count = 0;

public:
    /// @brief Should not be used often as it defeats the purpose of the reference governor
    /// @param reference State map setting the reference map (should equal the robots current estimate)
    void set_reference_map(RobotState& reference);

    /// @brief Sets the position reference for a given state
    /// @param value The value to set
    /// @param state_name The name of the state to set the reference for  
    /// @note This function should be used sparingly, as setting the reference defeats its purpose.
    void set_position_reference(float value, NewConfig::StateName state_name);

    /// @brief Sets the velocity reference for a given state
    /// @param value The value to set
    /// @param state_name The name of the state to set the reference for
    /// @note This function should be used sparingly, as setting the reference defeats its purpose.
    void set_velocity_reference(float value, NewConfig::StateName state_name);

    /// @brief Sets the acceleration reference for a given state
    /// @param value the value to set
    /// @param state_name The name of the state to get the reference for
    /// @note This function should be used sparingly, as setting the reference defeats its purpose.
    float set_acceleration_reference(float value, NewConfig::StateName state_name);

    /// @brief Gives the instantaneous governed state reference matrix (also known as desired state)
    /// @param reference The array to override with the reference matrix; Must be of shape [STATE_LEN][3]
    void get_reference_map(RobotState& reference);

    /// @brief Steps the reference matrix towards a goal, applying a reference governor to prevent impossible motion
    /// @param governor_type position based governor (1) or velocity based governor (2)
    void step_reference(const RobotState& ungoverned_reference_map);
};

#endif