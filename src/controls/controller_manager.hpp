#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "controller.hpp"
#include "../comms/rm_can.hpp"
#include "../sensors/RefSystem.hpp"

/// @brief Manage all controllers
class ControllerManager {
private:
    /// @brief Keep track of the controller used on each motor
    Controller* controllers[NUM_MOTORS][NUM_CONTROLLER_LEVELS];

public:
    /// @brief default constructor
    ControllerManager();

    /// @brief Populates the corresponding index of the "controllers" array attribute with a controller object
    /// @param can_id can bus number. Use the defines! (0 indexed. ie can_1 = 0)
    /// @param motor_id motor id (1 indexed. ie motor id 1 = 1)
    /// @param controller_type denotes what kind of controller to initialize (see contoller.hpp)
    /// @param controller_level denotes level of controller and what it outputs, such as a target micro state, or a target motor torque to send to can.
    /// @param gains gains matrix input (see controller.hpp for what each gain means)
    void init_controller(uint8_t can_id, uint8_t motor_id, int controller_type, int controller_level, float gains[NUM_GAINS]);

    /// @brief Steps through controllers and calculates output, which is written to the "output" array attribute.
    /// @param macro_reference State reference (governed target state)
    /// @param macro_estimate estimated current joint states
    /// @param micro_estimate estimated current motor states
    /// @param kinematics_p position kinematics matrix relating motors to states (Number_of_motors x State_length)
    /// @param kinematics_v velocity kinematics matrix relating motors to states (Number_of_motors x State_length)
    /// @param outputs generated motor input normalized -1 to 1
    void step(float macro_reference[STATE_LEN][3], float macro_estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float kinematics_p[NUM_MOTORS][STATE_LEN], float kinematics_v[NUM_MOTORS][STATE_LEN], float outputs[NUM_MOTORS]);

    /// @brief Stores motor outputs
    float output[NUM_MOTORS];

    /// @brief get the ratio (between 0 and 1) of power limit. 1 when 60 to 20 and x/20 under 20.
    /// @return ratio to scale the chassis control based on current power buffer
    float powerlimit_ratio();
};

#endif // CONTROLLER_MANAGER_H