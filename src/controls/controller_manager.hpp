#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "controller.hpp"
#include "sensors/can/can_manager.hpp"
#include "comms/config_layer.hpp"

#define MOTOR_INFO_TYPE 0
#define MOTOR_INFO_ID 1
#define MOTOR_INFO_BUS 2

/// @brief Manage all controllers
class ControllerManager {
private:

    std::vector<Controller> controllers;

    /// @brief array of motor outputs
    float outputs[CAN_MAX_MOTORS] = { 0 };

    /// @brief can data pointer used to write to the can bus
    CANManager* can;

public:
    /// @brief default constructor, does nothing
    ControllerManager() = default;

    /// @brief Initializes controllers with data from the config yaml
    /// @param _can pointer to the can data struct
    /// @param _config_data read-only config reference storing all config data
    void init(CANManager* _can, const std::vector<NewConfig::Controller>& controller_configs);

    /// @brief Populates the corresponding index of the "controllers" array attribute with a controller object
    /// @param controller_type denotes what kind of controller to initialize (see contoller.hpp)
    /// @param gains gains matrix input (see controller.hpp for what each gain means)
    /// @param gear_ratios gear ratios array, used to relate motor inputs to joint states
    void init_controller(int controller_type, const float gains[NUM_GAINS], const float gear_ratios[CAN_MAX_MOTORS]);

    /// @brief Steps through controllers and sets the new motor inputs (ie. motor current, torque)
    /// @param macro_reference Governor reference (governed target state)
    /// @param macro_estimate estimated current joint states
    /// @param micro_estimate estimated current motor states
    void step(float macro_reference[STATE_LEN][3], float macro_estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN]);

    /// @brief An abstracted method to write to a specific motor by ID
    /// @param motor_id The ID of a motor, these IDs are defined in the config
    /// @param value 
    void actuator_write(int motor_id, float value);
};
#endif // CONTROLLER_MANAGER_H