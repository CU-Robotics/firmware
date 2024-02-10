#include "controllers.hpp"
#include "../comms/rm_can.hpp"
#include "state.hpp"

#ifndef CONTROL_H
#define CONTROL_H

class Control {
    private:
        Control() = default;

        /// @brief Singleton instance
        static Control* instance;
        Controller* controllers[NUM_MOTORS];

    public:
        /// @brief Gives the singleton instance
        static Control* get_instance() {
            if(instance == nullptr) instance = new Control();
            return instance;
        }


        /// @brief Populates the corresponding index of the "controllers" array attribute with a controller object
        void init_controller(uint8_t can_id, uint8_t motor_id, int controller_type);

        /// @brief Steps through controllers and calculates output, which is written to the "output" array attribute.
        void step(float dt,float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float kinematics[NUM_MOTORS][STATE_LEN], float outputs[NUM_MOTORS]);

        /// @brief Stores motor outputs
        float output[NUM_MOTORS];

        /// @brief get the ratio (between 0 and 1) of power limit. 1 when 60 to 20 and x/20 under 20.
        float powerlimit_ratio ();
};

#endif // CONTROL_H