#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "./state.hpp"
#include "../sensors/dr16.hpp"
#include "../sensors/ICM20649.hpp"
#include "../sensors/IMUSensor.hpp"
#include "../sensors/LSM6DSOX.hpp"
#include "../sensors/rev_encoder.hpp"
#include "../sensors/buff_encoder.hpp"
#include "estimator.hpp"
#include "../comms/rm_can.hpp"
#include <SPI.h>

// maximum number of each sensor (arbitrary)
#define NUM_SENSOR_TYPE 16

#define NUM_IMU_CALIBRATION 50000

#define NUM_ESTIMATORS 2

//x y psi theta phi flywheel_l flywheel_r feeder switcher
class EstimatorManager {
private:
    /// @brief sensor arrays

    ICM20649 icm_sensors[NUM_SENSOR_TYPE];
    LSM6DSOX lsm_sensors[NUM_SENSOR_TYPE];
    // RevEncoder rev_sensors[NUM_SENSOR_TYPE];
    BuffEncoder buff_sensors[NUM_SENSOR_TYPE];

    /// @brief Singleton instance
    Estimator* estimators[STATE_LEN] = {nullptr};

    int applied_states[NUM_ESTIMATORS][STATE_LEN];

    CANData *can_data;

public:
    /// @brief initialize sensors and set can_data pointer
    /// @param data Struct storing all of can data so we don't have to pass around rmCAN itself.
    EstimatorManager(CANData *data);

    /// @brief Free all dynamically allocated memory and end SPI
    ~EstimatorManager();

    /// @brief Populates the corresponding index of the "estimators" array attribute with an estimator object.
    void init_estimator(int state_id,int num_states);

    /// @brief Steps through every estimator and constructs a state array based on current sensor values.
    /// @param state State array pointer to be updated.
    void step(float state[STATE_LEN][3]);

    /// @brief read all sensor arrays besides can and dr16(they are in main).
    void read_sensors();

    void calibrate_imus();

    void assign_states(int as [NUM_ESTIMATORS][STATE_LEN]);
};

#endif