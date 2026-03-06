#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "state.hpp"
#include "estimator.hpp"
#include "sensors/sensor_manager.hpp"

/// @brief Manage all estimators for macro and micro state
class EstimatorManager {
private:
    std::vector<std::unique_ptr<Estimator>> estimators;

    std::vector<Cfg::StateName> available_states;
public:
    /// @brief Assign references to the can manager and sensor manager
    EstimatorManager();

    /// @brief Free all dynamically allocated memory and end SPI
    ~EstimatorManager();

    /// @brief initialize all sensors and set can_data pointer
    /// @param can CAN manager pointer to get access to motor state
    /// @param config_data read only reference struct storing all the config data
    /// @param sensor_manager pointer to the sensor manager to read sensor data
    void init(const std::vector<Cfg::Estimator>& estimator_configurations, SensorManager& sensor_manager, CANManager& can);

    /// @brief Steps through every estimator and constructs a state array based on current sensor values.
    /// @param state macro state array pointer to be updated.
    /// @param micro_state micro state array pointer to be updated.
    /// @param override true if we want to override the current state with the new state.
    void step(RobotStateMap& updated_state_map, int override);
    
private:
    void init_estimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can);
};

#endif