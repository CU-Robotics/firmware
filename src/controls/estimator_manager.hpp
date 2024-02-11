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

    CANData *can_data;

public:
    EstimatorManager(CANData *data);
    ~EstimatorManager();

    /// @brief Populates the corresponding index of the "estimators" array attribute with an estimator object
    void init_estimator(int state_id);

    /// @brief Steps through estimators and calculates current state, which is written to the "state" array attribute
    void step(float state[STATE_LEN][3]);

    void read_sensors();
};

#endif