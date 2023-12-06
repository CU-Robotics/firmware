#ifndef CONTROL_MANAGER_HPP
#define CONTROL_MANAGER_HPP

#include "rm_CAN.hpp"

/// @brief enums for each control type
enum control_types {
    PID = 0,
    // ... the rest of the control types
};

class control_manager {
public:
    control_manager(rm_CAN* _can);

    /// @brief updates all of the torques for each motor based on the yaml file
    void update_motors();

private: // members

    /// @brief The state we want each motor to be
    float set_state[NUM_CANS][NUM_MOTORS];

    /// @brief the current state as read by the can
    float curr_state[NUM_CANS][NUM_MOTORS];
    
    /// @brief what kind of control theory we want to use on each motor
    int control_type[NUM_CANS][NUM_MOTORS];

    /// @brief the amplitude of torque we write to each motor
    float gains[NUM_CANS][NUM_MOTORS];

    /// @brief the final output to write to each motor
    float output[NUM_CANS][NUM_MOTORS];

    /// @brief can instance to write to motors
    rm_CAN* can;

private: // methods

    /// @brief updates each motors output based on set control type
    void step_motors();

    // all of the different kinds of
    // controller algorithms

    /// @brief updates a motors output based on PID
    /// @param curr current state of the motor
    /// @param set set state of the motor
    float step_PID(float curr, float set);
    // ...

};

#endif // CONTROL_MANAGER_HPP