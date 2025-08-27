#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "state.hpp"
#include "sensors/dr16.hpp"
#include "sensors/ICM20649.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/LSM6DSOX.hpp"
#include "sensors/rev_encoder.hpp"
#include "sensors/TOFSensor.hpp"
#include "sensors/buff_encoder.hpp"
#include "sensors/RefSystem.hpp"
#include "estimator.hpp"
#include <SPI.h>
#include "../sensors/SensorManager.hpp"

// maximum number of each sensor (arbitrary)
#define NUM_SENSOR_TYPE 16

#define NUM_IMU_CALIBRATION 50000

/// @brief Manage all estimators for macro and micro state
class EstimatorManager {
private:

    /// @brief array of robot estimators to estimate full robot state
    Estimator* estimators[STATE_LEN] = { nullptr };

    /// @brief can pointer to pass to each estimator so they can use can to estimate state when needed (usually used for micro state).
    CANManager* can = nullptr;

    /// @brief pointer to the sensor manager to read sensor data
    SensorManager* sensor_manager;

    /// @brief config struct to store all config data
    /// @note this is read only
    const Config* config_data = nullptr;

    /// @brief current number of estimators
    int num_estimators = 0;

public:
    /// @brief Default constructor, does nothing
    EstimatorManager() = default;

    /// @brief Free all dynamically allocated memory and end SPI
    ~EstimatorManager();

    /// @brief initialize all sensors and set can_data pointer
    /// @param can CAN manager pointer to get access to motor state
    /// @param config_data read only reference struct storing all the config data
    /// @param sensor_manager pointer to the sensor manager to read sensor data
    void init(CANManager* can, const Config* config_data, SensorManager* sensor_manager);

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

    /// @brief Populates the corresponding index of the "estimators" array attribute with an estimator object.
    /// @param estimator_id id of estimator to init
    void init_estimator(int estimator_id);

    /// @brief sets the assigned states array use for telling which estimators estimate which states
    /// @param as assigned array
    void assign_states(const float as[NUM_ESTIMATORS][STATE_LEN]);
};

#endif