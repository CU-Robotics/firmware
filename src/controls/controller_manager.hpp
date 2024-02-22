#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "controller.hpp"
#include "../comms/rm_can.hpp"
#include "state.hpp"

#define NUM_CONTROLLERS 1
class ControllerManager {
    private:
        Controller* controllers[NUM_MOTORS];

    public:
        ControllerManager();

        /// @brief Populates the corresponding index of the "controllers" array attribute with a controller object
        /// @param can_id can bus number. Use the defines! (0 indexed. ie can_1 = 0)
        /// @param motor_id motor id (1 indexed. ie motor id 1 = 1)
        /// @param controller_type denotes what kind of controller to initialize (see contoller.hpp)
        /// @param gains gains matrix input (see controller.hpp for what each gain means)
        void init_controller(uint8_t can_id, uint8_t motor_id, int controller_type, float gains[NUM_GAINS]);

        /// @brief Steps through controllers and calculates output, which is written to the "output" array attribute.
        /// @param reference State reference (governed target state)
        /// @param estimate estimated current state
        /// @param kinematics kinematics matrix relating motors to states (Number_of_motors x State_length)
        /// @param outputs generated motor input normalized -1 to 1
        void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float kinematics_p[NUM_MOTORS][STATE_LEN], float kinematics_v[NUM_MOTORS][STATE_LEN], float outputs[NUM_MOTORS]);

        /// @brief Stores motor outputs
        float output[NUM_MOTORS];

        /// @brief get the ratio (between 0 and 1) of power limit. 1 when 60 to 20 and x/20 under 20.
        float powerlimit_ratio();
};

#endif // CONTROLLER_MANAGER_H