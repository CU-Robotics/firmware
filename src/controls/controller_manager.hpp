#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "controller.hpp"
#include "../comms/rm_can.hpp"
#include "../sensors/RefSystem.hpp"
#include "../comms/config_layer.hpp"

/// @brief Manage all controllers
class ControllerManager {
private:
    /// @brief Array storing every controller
    Controller* controllers[NUM_ROBOT_CONTROLLERS];
    
    int num_controllers = 0;

    float outputs[NUM_MOTORS] = { 0 };

    /// @brief config struct to store all config data
    /// @note this is read only
    const Config* config_data = nullptr;

    /// @brief can data pointer used to write to the can bus
    rm_CAN* can;

public:
    /// @brief default constructor, does nothing
    ControllerManager() = default;

    /// @brief Initializes controllers with data from the config yaml
    /// @param _config_data read-only config reference storing all config data
    void init(rm_CAN* _can, const Config* _config_data);

    /// @brief Populates the corresponding index of the "controllers" array attribute with a controller object
    /// @param controller_type denotes what kind of controller to initialize (see contoller.hpp)
    /// @param gains gains matrix input (see controller.hpp for what each gain means)
    void init_controller(int controller_type, const float gains[NUM_GAINS], const float gear_ratios[NUM_MOTORS]);

    /// @brief Steps through controllers and sets the new motor inputs (ie. motor current, torque)
    /// @param macro_reference Governor reference (governed target state)
    /// @param macro_estimate estimated current joint states
    /// @param micro_estimate estimated current motor states
    void step(float macro_reference[STATE_LEN][3], float macro_estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN]);

    /// @brief get the ratio (between 0 and 1) of power limit. 1 when 60 to 20 and x/20 under 20.
    /// @return ratio to scale the chassis control based on current power buffer
    float powerlimit_ratio();

    /// @brief An abstracted method to write to a specific motor by ID
    /// @param motor_id The ID of a motor, these IDs are defined in the config
    /// @param value 
    void actuator_write(int motor_id, float value);
};
#endif // CONTROLLER_MANAGER_H