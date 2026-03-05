#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "controller.hpp"
#include "sensors/can/can_manager.hpp"
#include "comms/config_layer.hpp"

/// @brief Manage all controllers
class ControllerManager {
private:
    /// @brief List of all controllers that are currently active
    std::vector<std::unique_ptr<Cfg::Controller>> controllers;

    /// @brief List of motors that are available to be used by controllers. 
    //  This is used to make sure that two controllers cant write to the same motor
    std::vector<Cfg::MotorName> available_motors;
public:
    /// @brief assign reference to the can manager
    ControllerManager() {}

    /// @brief Initializes controllers with data from the config yaml
    /// @param _can reference to the can data struct
    /// @param _config_data read-only config reference storing all config data
    void init(const std::vector<Cfg::Controller>& controller_configurations, CANManager& _can);

    /// @brief Initializes and adds a new controller to the controller manager
    /// @param controller_config the controller configuration data to use to initialize the controller
    void init_controller(const Cfg::Controller& controller_config, CANManager& _can);
};
#endif // CONTROLLER_MANAGER_H