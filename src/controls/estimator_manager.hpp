#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "state.hpp"
#include "sensors/rev_encoder.hpp"
#include "estimator.hpp"
#include <SPI.h>
#include "../sensors/SensorManager.hpp"

// maximum number of each sensor (arbitrary)
#define NUM_SENSOR_TYPE 16

#define NUM_IMU_CALIBRATION 50000

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
    void init(const std::vector<Cfg::Estimator>& estimator_configurations, const CANManager& can, const SensorManager& sensor_manager);

    /// @brief Steps through every estimator and constructs a state array based on current sensor values.
    /// @param state macro state array pointer to be updated.
    /// @param micro_state micro state array pointer to be updated.
    /// @param override true if we want to override the current state with the new state.
    void step(RobotStateMap updated_state_map, RobotStateMap previous_state_map, int override);
    
private:
    void init_estimator(const Cfg::Estimator& estimator_config, const CANManager& can, const SensorManager& sensor_manager);
};

#endif