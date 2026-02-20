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
    std::vector<std::unique_ptr<Estimator>> high_level_estimators;
    std::vector<std::unique_ptr<Estimator>> low_level_estimators;

public:
    /// @brief Assign references to the can manager and sensor manager
    EstimatorManager();

    /// @brief Free all dynamically allocated memory and end SPI
    ~EstimatorManager();

    /// @brief initialize all sensors and set can_data pointer
    /// @param can CAN manager pointer to get access to motor state
    /// @param config_data read only reference struct storing all the config data
    /// @param sensor_manager pointer to the sensor manager to read sensor data
    void init(const std::vector<NewConfig::Estimator>& estimator_configurations);

    /// @brief Steps through every estimator and constructs a state array based on current sensor values.
    /// @param state macro state array pointer to be updated.
    /// @param micro_state micro state array pointer to be updated.
    /// @param override true if we want to override the current state with the new state.
    void step(float state[STATE_LEN][3], float micro_state[CAN_MAX_MOTORS][MICRO_STATE_LEN], int override);

    /// @brief sets both input arrays to all 0's
    /// @param macro_outputs input 1
    /// @param micro_outputs input 2
    void clear_outputs(float macro_outputs[STATE_LEN][3], float micro_outputs[CAN_MAX_MOTORS][MICRO_STATE_LEN]);


private:
    void init_estimator(const NewConfig::HighLevelEstimator& estimator_config, const CANManager& can, const SensorManager& sensor_manager);

    void init_estimator(const NewConfig::LowLevelEstimator& estimator_config, const CANManager& can, const SensorManager& sensor_manager);

    /// @brief sets the assigned states array use for telling which estimators estimate which states
    /// @param as assigned array
    void assign_states(const float as[NUM_ESTIMATORS][STATE_LEN]);
};

#endif