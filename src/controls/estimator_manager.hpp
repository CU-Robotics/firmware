#pragma once

#include "state.hpp"
#include "estimator.hpp"
#include "sensors/sensor_manager.hpp"

/// @brief Manage all estimators for macro and micro state
class EstimatorManager {
private:
    /// @brief List of all estimators that are currently active
    std::vector<std::unique_ptr<Estimator>> estimators;
    /// @brief List of states that estimators can claim
    std::vector<Cfg::StateName> available_states;
public:
    /// @brief Assign references to the can manager and sensor manager
    EstimatorManager();

    /// @brief clear the list of estimators
    ~EstimatorManager();

    /// @brief Initializes estimators with data from the config
    /// @param estimator_configurations vector of estimator configurations
    /// @param sensor_manager reference to the sensor manager so estimators can grab their sensors
    /// @param can CAN manager reference so estimators can grab their motors
    void init(const std::vector<Cfg::Estimator>& estimator_configurations, SensorManager& sensor_manager, CANManager& can);

    /// @brief Steps through every estimator to update the state estimate
    /// @param updated_state_map the current state estimate map, which is updated by each estimator as they step through
    /// @param override whether we are currently overriding the estimated state map
    void step(RobotStateMap& updated_state_map, int override);
    
private:
    /// @brief init an estimator based on the estimator configuration, and add it to the estimator manager's list of estimators
    /// @param estimator_config configuration data for the estimator to be initialized
    /// @param sensor_manager reference to the sensor manager so the estimator can grab its sensors
    /// @param can CAN manager reference so the estimator can grab its motors
    void init_estimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can);
};