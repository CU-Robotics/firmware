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
    CANManager& can;

public:
    /// @brief assign reference to the can manager
    ControllerManager(CANManager& _can) : can(_can) {}

    /// @brief Initializes controllers with data from the config yaml
    /// @param _can reference to the can data struct
    /// @param _config_data read-only config reference storing all config data
    void init(const std::vector<NewConfig::Controller>& controller_configurations);

    /// @brief Initializes and adds a new controller to the controller manager
    /// @param controller_config the controller configuration data to use to initialize the controller
    void init_controller(NewConfig::Controller controller_config);

    /// @brief Steps through controllers and sets the new motor inputs (ie. motor current, torque)
    /// @param macro_reference Governor reference (governed target state)
    /// @param macro_estimate estimated current joint states
    /// @param micro_estimate estimated current motor states
    void step(float macro_reference[STATE_LEN][3], float macro_estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN]);
};
#endif // CONTROLLER_MANAGER_H