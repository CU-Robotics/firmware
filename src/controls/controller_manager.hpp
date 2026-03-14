#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "controller.hpp"
#include "robot_state_map.hpp"
#include "sensors/can/can_manager.hpp"
#include <memory>
#include "comms/config_layer.hpp"

/// @brief Manage all controllers
class ControllerManager {
private:
    /// @brief List of all controllers that are currently active
    std::vector<std::unique_ptr<Controller>> controllers;

    /// @brief List of motors that are available to be used by controllers. 
    //  This is used to make sure that two controllers cant write to the same motor
    std::vector<Cfg::MotorName> available_motors;
public:
    /// @brief assign reference to the can manager
    ControllerManager() {}

    /// @brief Initializes controllers with data from the config yaml
    /// @param controller_configurations read-only config reference storing all config data
    /// @param _can reference to the can data struct
    void init(const std::vector<Cfg::Controller>& controller_configurations, CANManager& _can);

    /// @brief Initializes and adds a new controller to the controller manager
    /// @param controller_config the controller configuration data to use to initialize the controller
    /// @param _can reference to the can data struct to use to initialize the controller
    void init_controller(const Cfg::Controller& controller_config, CANManager& _can);

    /// @brief Steps all controllers in the controller manager. This should be called every control loop iteration
    /// @param reference_map the map of reference states that controllers should try to achieve
    /// @param estimate_map the map of estimated states that controllers should use to calculate their outputs
    void step(RobotStateMap& reference_map, RobotStateMap& estimate_map);
};
#endif // CONTROLLER_MANAGER_H