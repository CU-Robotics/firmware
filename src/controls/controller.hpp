#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

//THESE VALUES MUST MATCH YAML
#define WEIGHTS_LEN 12
#define NUM_MOTORS 16
#define CAN1_LEN 6
#define CAN2_LEN 6

class Controller {
public:
    /// @brief instantiate a controller which handles control for all motors. There should only be one one instance of this class.
    /// @param _controller_weights 2D array to be copied over as a member of this class using memcpy
    /// @param _controller_types 1D array to be copied over as a member of this class using memcpy
    /// @param _kinematics 3D array to be copied over as a member of this class using memcpy
    /// @param weights_size size of _controller_weights array in bytes
    /// @param types_size size of _controller_types array in bytes
    /// @param kinematics_size size of _kinematics array in bytes
    Controller(float _controller_weights[NUM_MOTORS][WEIGHTS_LEN], int _controller_types[NUM_MOTORS], float _kinematics[NUM_MOTORS][STATE_LEN], int weights_size, int types_size, int kinematics_size);

    /// @brief cycles through all motors and calculates control outputs for each motor using the stepper functions and the kinematics, controller_weights, and controller_types arrays. 
    /// @param input array to be updated with outputs from controls
    void step_controllers(float input[NUM_MOTORS]);

private: // members
    /// @brief 2D array which stores our target state in format [position, velocity, acceleration] for each state. Updated from the State singleton every time step_controllers() is called.
    float reference[STATE_LEN][3];

    /// @brief 2D array which stores our current estimated state in the same format as reference. Updated from the State singleton every time step_controllers() is called.
    float estimate[STATE_LEN][3];

    /// @brief 1D array which stores the motor outputs we calculate. This array is returned when step_controllers() is called via memcpy.
    float outputs[NUM_MOTORS];

    /// @brief 2D array which stores important control data for each motor while running control loops. For example, when running PID, this array stores delta error at control_data[n][0], total error sum at control_data[n][1]
    float control_data[NUM_MOTORS][WEIGHTS_LEN];

    /// @brief 3D array [CAN][MOTOR_ID][STATE] which stores the kinematics on each state based off of can bus and motor id. Updated from the Configuration singleton every time step_controllers() is called.
    float kinematics[2][NUM_MOTORS][STATE_LEN];
    
    /// @brief 2D array which stores the control gains for each motor (see configuration.hpp). Updated from the Configuration singleton every time step_controllers() is called.
    float controller_weights[NUM_MOTORS][WEIGHTS_LEN];

    /// @brief 1D array which stores the type of control each motor uses. Updated from the Configuration singleton every time step_controllers() is called.
    int controller_types[NUM_MOTORS];


private: // methods
    /// @brief stepper for a pid position controller
    /// @param motor_id motor index we this stepper is applied to so we can access the right control data.
    /// @param reference target state array [position, velocity, acceleration]
    /// @param estimate current state array [position, velocity, acceleration]
    /// @return stepper output
    float step_PID_position(int motor_id, float reference[3], float estimate[3]);

    /// @brief stepper for a pid velocity controller
    /// @param motor_id motor index we this stepper is applied to so we can access the right control data.
    /// @param reference target state array [position, velocity, acceleration]
    /// @param estimate current state array [position, velocity, acceleration]
    /// @return stepper output
    float step_PID_velocity(int motor_id, float reference[3], float estimate[3]);
    // ...

};

#endif