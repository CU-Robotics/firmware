#include "utils/timing.hpp"
#include "controls/robot_state_map.hpp"

#ifndef STATE_H
#define STATE_H

/// @brief Use reference limits from config to convert ungoverned reference states to generated governed reference states to be sent to controllers.
class Governor {
private:
    // This is a sample state (it does not represent every robot):
    // {x, y, psi (chassis angle), theta (yaw angle), phi (pitch angle), feed, flywheel}
    // In this example case, as with all other cases, the unused state rows are kept blank.

    RobotStateMap reference_state_map;

    /// @brief Timer for the reference governor
    Timer governor_timer;

    /// @brief counter so dt isnt big in the first loop
    int count = 0;

public:

    Governor(std::vector<Cfg::State> state_configurations) : reference_state_map(state_configurations) {}

    /// @brief Should not be used often as it defeats the purpose of the reference governor
    /// @param reference State map setting the reference map (should equal the robots current estimate)
    void set_reference_map(const RobotStateMap& new_reference);

    /// @brief Sets the position reference for a given state
    /// @param value The value to set
    /// @param state_name The name of the state to set the reference for  
    /// @note This function should be used sparingly, as setting the reference defeats its purpose.
    void set_position_reference(Cfg::StateName state_name, float value);

    /// @brief Sets the velocity reference for a given state
    /// @param value The value to set
    /// @param state_name The name of the state to set the reference for
    /// @note This function should be used sparingly, as setting the reference defeats its purpose.
    void set_velocity_reference(Cfg::StateName state_name, float value);

    /// @brief Sets the acceleration reference for a given state
    /// @param value the value to set
    /// @param state_name The name of the state to get the reference for
    /// @note This function should be used sparingly, as setting the reference defeats its purpose.
    void set_acceleration_reference(Cfg::StateName state_name, float value);

    /// @brief Gives the instantaneous governed state reference matrix (also known as desired state)
    /// @param reference The array to override with the reference matrix; Must be of shape [STATE_LEN][3]
    const RobotStateMap& get_reference_map() const;

    /// @brief Steps the reference matrix towards a goal, applying a reference governor to prevent impossible motion
    /// @param governor_type position based governor (1) or velocity based governor (2)
    const RobotStateMap& step_reference_map(const RobotStateMap& ungoverned_reference_map);
};

#endif