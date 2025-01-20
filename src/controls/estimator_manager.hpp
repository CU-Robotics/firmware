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
#include "comms/rm_can.hpp"
#include <SPI.h>

// maximum number of each sensor (arbitrary)
#define NUM_SENSOR_TYPE 16

#define NUM_IMU_CALIBRATION 50000

/// @brief Manage all estimators for macro and micro state
class EstimatorManager {
private:
    /// @brief array to store robot icm imu's
    ICM20649 icm_sensors[NUM_SENSOR_TYPE];
    /// @brief array to store robot buff encoders
    BuffEncoder buff_encoders[NUM_SENSOR_TYPE];
    /// @brief array to store robot rev encoders
    RevEncoder rev_sensors[NUM_SENSOR_TYPE];
    /// @brief array to store tof sensors
    TOFSensor tof_sensors[NUM_SENSOR_TYPE];

    /// @brief array of robot estimators to estimate full robot state
    Estimator* estimators[STATE_LEN] = { nullptr };

    /// @brief can pointer to pass to each estimator so they can use can to estimate state when needed (usually used for micro state).
    CANManager* can = nullptr;

    /// @brief config struct to store all config data
    /// @note this is read only
    const Config* config_data = nullptr;

    /// @brief current number of estimators
    int num_estimators = 0;

    /// @brief array to store the number of sensors for each sensor type
    int num_sensors[NUM_SENSORS];

public:
    /// @brief Default constructor, does nothing
    EstimatorManager() = default;

    /// @brief Free all dynamically allocated memory and end SPI
    ~EstimatorManager();

    /// @brief initialize all sensors and set can_data pointer
    /// @param can reference struct storing all of can data so we dont have to pass rm_can around
    /// @param config_data read only reference struct storing all the config data
    void init(CANManager* can, const Config* config_data);

    /// @brief Steps through every estimator and constructs a state array based on current sensor values.
    /// @param state macro state array pointer to be updated.
    /// @param micro_state micro state array pointer to be updated.
    /// @param override true if we want to override the current state with the new state.
    void step(float state[STATE_LEN][3], float micro_state[CAN_MAX_MOTORS][MICRO_STATE_LEN], int override);

    /// @brief read all sensor arrays besides can and dr16(they are in main).
    void read_sensors();

    /// @brief sets both input arrays to all 0's
    /// @param macro_outputs input 1
    /// @param micro_outputs input 2
    void clear_outputs(float macro_outputs[STATE_LEN][3], float micro_outputs[CAN_MAX_MOTORS][MICRO_STATE_LEN]);

    /// @brief get the specified buff encoder sensor from the array
    /// @param index index of the sensor object to get
    /// @return reference to the buff encoder sensor
    BuffEncoder& get_buff_encoders(int index) {
        return buff_encoders[index];

    }

    /// @brief get the specified icm sensor from the array
    /// @param index index of the sensor object to get
    /// @return reference to the icm sensor
    ICM20649& get_icm_sensors(int index) {
        return icm_sensors[index];
    }

    /// @brief get the specified rev sensor from the array
    /// @param index index of the sensor object to get
    /// @return reference to the rev sensor
    RevEncoder& get_rev_sensors(int index) {
        return rev_sensors[index];
    }

    /// @brief get the specified tof sensor from the array
    /// @param index index of the sensor object to get
    /// @return reference to the tof sensor
    TOFSensor& get_tof_sensors(int index) {
        return tof_sensors[index];
    }
    
private:
    /// @brief call read for imu's NUM_IMU_CALIBRATION times and then averages returns to calculate offset.
    void calibrate_imus();

    /// @brief Populates the corresponding index of the "estimators" array attribute with an estimator object.
    /// @param estimator_id id of estimator to init
    void init_estimator(int estimator_id);

    /// @brief sets the assigned states array use for telling which estimators estimate which states
    /// @param as assigned array
    void assign_states(const float as[NUM_ESTIMATORS][STATE_LEN]);
};

#endif