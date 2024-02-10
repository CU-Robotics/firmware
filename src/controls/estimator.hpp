#include "./state.hpp"
#include "../sensors/dr16.hpp"
#include "../sensors/ICM20649.hpp"
#include "../sensors/IMUSensor.hpp"
#include "../sensors/LSM6DSOX.hpp"
#include "../sensors/rev_encoder.hpp"
#include "../sensors/buff_encoder.hpp"
#include "estimators.hpp"
#include "../comms/rm_can.hpp"
#include <SPI.h>

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

// maximum number of each sensor (arbitrary)
#define NUM_SENSOR_TYPE 16

//x y psi theta phi flywheel_l flywheel_r feeder switcher
class EstimatorManager {
    private:
        EstimatorManager() = default;

        /// @brief sensor arrays
        rm_CAN *can;
        ICM20649 icm_sensors[NUM_SENSOR_TYPE];
        LSM6DSOX lsm_sensors[NUM_SENSOR_TYPE];
        // RevEncoder rev_sensors[NUM_SENSOR_TYPE];
        BuffEncoder buff_sensors[NUM_SENSOR_TYPE];

        // ISAAC COMMENTS: Make the above more descriptive with a word that shows that these all do the same thing (icm_sensors. rev_sensors, etc)
        // Get rid of IMUSensor, dont need that one
        // Looks really fucking good overall. Maybe think about the pitch estimator logic a little more, I think there are some edge cases
       
        /// @brief Singleton instance
        Estimator* estimators[STATE_LEN];

        float output[STATE_LEN][3];

    public:
        /// @brief Gives the singleton instance
        static EstimatorManager* get_instance() {
            static EstimatorManager* instance;
            return instance;
        }

        void init();

        /// @brief Populates the corresponding index of the "estimators" array attribute with an estimator object
        void init_estimator(int state_id);

        /// @brief Steps through estimators and calculates current state, which is written to the "state" array attribute
        void step(float state[STATE_LEN][3]);


        void read_sensors();
};
#endif