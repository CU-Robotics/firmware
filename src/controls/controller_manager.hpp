#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "controller.hpp"
#include "../comms/rm_can.hpp"
#include "state.hpp"

class ControllerManager {
    private:
        ControllerManager() = default;
        Controller* controllers[NUM_MOTORS];

    public:
        /// @brief Populates the corresponding index of the "controllers" array attribute with a controller object
        void init_controller(uint8_t can_id, uint8_t motor_id, int controller_type);

        /// @brief Steps through controllers and calculates output, which is written to the "output" array attribute.
        void step(float dt,float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float kinematics[NUM_MOTORS][STATE_LEN], float outputs[NUM_MOTORS]);

        /// @brief Stores motor outputs
        float output[NUM_MOTORS];

        /// @brief get the ratio (between 0 and 1) of power limit. 1 when 60 to 20 and x/20 under 20.
        float powerlimit_ratio();
};

#endif // CONTROLLER_MANAGER_H