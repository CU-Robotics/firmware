#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "./state.hpp"
#include "../sensors/dr16.hpp"
#include "../sensors/ICM20649.hpp"
#include "../sensors/IMUSensor.hpp"
#include "../sensors/LSM6DSOX.hpp"
#include "../sensors/rev_encoder.hpp"
#include "../sensors/TOFSensor.hpp"
#include "../sensors/buff_encoder.hpp"
#include "../sensors/RefSystem.hpp"
#include "estimator.hpp"
#include "../comms/rm_can.hpp"
#include "../comms/config_layer.hpp"
#include <SPI.h>

// maximum number of each sensor (arbitrary)
#define NUM_SENSOR_TYPE 16

#define NUM_IMU_CALIBRATION 50000


// Rev encoder pins
#define REV_ENC_PIN1 2
#define REV_ENC_PIN2 3
#define REV_ENC_PIN3 4

/// @brief Manage all estimators for macro and micro state
class EstimatorManager {
private:
    //sensor arrays

    /// @brief array to store robot icm imu's
    ICM20649 icm_sensors[NUM_SENSOR_TYPE];
    /// @brief array to store robot buff encoders
    BuffEncoder buff_sensors[NUM_SENSOR_TYPE];
    /// @brief array to store robot rev encoders
    RevEncoder rev_sensors[NUM_SENSOR_TYPE];
    /// @brief array to store tof sensors
    TOFSensor tof_sensors[NUM_SENSOR_TYPE];

    /// @brief array of robot estimators to estimate full robot state
    Estimator* estimators[STATE_LEN] = { nullptr };

    /// @brief matrix that is number_of_estimators by State_length.
    /// the values inside the matrix tell the estimator stepper which states to write to for each estimator
    int applied_states[NUM_ESTIMATORS][STATE_LEN];

    /// @brief can data pointer to pass to each estimator so they can use can to estimate state when needed (usually used for micro state).
    CANData* can_data;

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
    /// @param can_data reference struct storing all of can data so we dont have to pass rm_can around
    /// @param config_data read only reference struct storing all the config data
    void init(CANData* can_data, const Config* config_data);

    /// @brief Steps through every estimator and constructs a state array based on current sensor values.
    /// @param state macro state array pointer to be updated.
    /// @param micro_state micro state array pointer to be updated.
    /// @param override true if we want to override the current state with the new state.
    void step(float state[STATE_LEN][3], float micro_state[NUM_MOTORS][MICRO_STATE_LEN], int override);

    /// @brief read all sensor arrays besides can and dr16(they are in main).
    void read_sensors();

    /// @brief sets both input arrays to all 0's
    /// @param macro_outputs input 1
    /// @param micro_outputs input 2
    void clear_outputs(float macro_outputs[STATE_LEN][3], float micro_outputs[NUM_MOTORS][MICRO_STATE_LEN]);

private:
    /// @brief call read for imu's NUM_IMU_CALIBRATION times and then averages returns to calculate offset.
    void calibrate_imus();

    /// @brief Populates the corresponding index of the "estimators" array attribute with an estimator object.
    /// @param estimator_id id of estimator to init
    /// @param num_states number of states this estimator should estimate
    void init_estimator(int estimator_id, int num_states);

    /// @brief sets the assigned states array use for telling which estimators estimate which states
    /// @param as assigned array
    void assign_states(const float as[NUM_ESTIMATORS][STATE_LEN]);
};


#endif