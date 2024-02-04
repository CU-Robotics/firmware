#include "./state.hpp"
#include "../sensors/dr16.hpp"
#include "../sensors/ICM20649.hpp"
#include "../sensors/IMUSensor.hpp"
#include "../sensors/LSM6DSOX.hpp"
#include "../sensors/rev_encoder.hpp"

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

// maximum number of each sensor (arbitrary)
#define NUM_SENSOR_TYPE 16

class EstimatorManager {
    private:
        /// @brief sensor arrays
        DR16 dr16[NUM_SENSOR_TYPE];
        ICM20649 icm[NUM_SENSOR_TYPE];
        IMUSensor imu[NUM_SENSOR_TYPE];
        LSM6DSOX lsm[NUM_SENSOR_TYPE];
        RevEncoder rev[NUM_SENSOR_TYPE];

        /// @brief Singleton instance
        static EstimatorManager* instance;
        Estimator estimators[STATE_LEN];

        // 

    public:
        /// @brief Gives the singleton instance
        static EstimatorManager* get_instance() {
            if (instance == NULL) {
                instance = new EstimatorManager();
                return instance;
            } else return instance;
        }

        /// @brief Populates the corresponding index of the "estimators" array attribute with an estimator object
        void init_estimator(int state_id, int estimator_type);

        /// @brief Steps through estimators and calculates current state, which is written to the "state" array attribute
        void step()


}