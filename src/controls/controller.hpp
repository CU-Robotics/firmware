#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "filters/pid_filter.hpp"
#include "utils/timing.hpp"
#include "sensors/can/can_manager.hpp"
#include "state.hpp"
#include "robot_state_map.hpp"

#include "comms/config_data/controller.hpp"

#define NUM_GAINS 24
#define NUM_ROBOT_CONTROLLERS 12

/// @brief Parent controller struct, all controllers should be based off of this.
struct Controller {
protected:
    /// @brief config data for this specific controller
    const NewConfig::Controller& controller_config;

    /// @brief Timer object so we can use dt in controllers
    Timer timer;

public:
    /// @brief default constructor
    Controller(const NewConfig::Controller& _controller_config) : controller_config(_controller_config) { };

    /// @brief Generates an output from a state reference and estimation
    /// @param reference reference (target state)
    /// @param estimate current macro state estimate
    /// @param micro_estimate current micro state estimate
    /// @param outputs array of new motor inputs
    virtual void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {}

    /// @brief Resets integrators/timers
    virtual void reset() { timer.start(); }

    const NewConfig::Controller& config() const { return controller_config; }
};

/// @brief Position controller for the chassis
struct XDrivePositionController : public Controller {
private:

    NewConfig::SubController xy_position_controller;
    NewConfig::SubController xy_velocity_controller;
    NewConfig::SubController chassis_angle_controller;
    NewConfig::SubController chassis_angular_velocity_controller;
    NewConfig::SubController low_level_velocity_controller;
    NewConfig::SubController power_buffer_controller;

    /// @brief filter for calculating pid position controller outputs. 3 for x, y, and chassis angle
    PIDFilter pidp[3];
    /// @brief filter for calculating pid velocity controller outputs. 3 for x, y, and chassis angle
    PIDFilter pidv[3];
    /// @brief outputs of the pid position controller
    float outputp[4];
    /// @brief outputs of the pid velocity controller
    float outputv[4];
    /// @brief combined outputs of the pid position and velocity controllers
    float output[4];
    /// @brief target motor velocity
    float motor_velocity[4];
    
    const C620& chassis_motor_1;
    const C620& chassis_motor_2;
    const C620& chassis_motor_3;
    const C620& chassis_motor_4;

public:
    /// @brief default
    XDrivePositionController() {
        xy_position_controller = controller_config.get_sub_controller_by_type(NewConfig::XYPositionController);
        xy_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::XYVelocityController);
        chassis_angle_controller = controller_config.get_sub_controller_by_type(NewConfig::ChassisAngleController);
        chassis_angular_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::ChassisAngularVelocityController);
        low_level_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::LowLevelVelocityController);
        power_buffer_controller = controller_config.get_sub_controller_by_type(NewConfig::PowerBufferController);
    }

    /// @brief calculate motor outputs based on reference and estimate
    /// @param reference current target robot state
    /// @param estimate current estimate robot state
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs motor outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);
    // New overload using RobotStateMap
    void step(RobotStateMap reference_map, RobotStateMap estimate_map, float outputs[CAN_MAX_MOTORS]);

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        for (int i = 0; i < 3; i++) {
            pidp[i].sumError = 0.0;
            pidv[i].sumError = 0.0;
        }
    }
};

/// @brief Controller for all chassis movement, which includes power limiting
struct XDriveVelocityController : public Controller {
private:
    NewConfig::SubController xy_velocity_controller;
    NewConfig::SubController chassis_angle_controller;
    NewConfig::SubController chassis_angular_velocity_controller;
    NewConfig::SubController low_level_velocity_controller;
    NewConfig::SubController power_buffer_controller;

    /// @brief filter for calculating pid position controller outputs. 3 for x, y, and chassis angle
    PIDFilter pidp[3];
    /// @brief filter for calculating pid velocity controller outputs. 3 for x, y, and chassis angle
    PIDFilter pidv[3];
    /// @brief outputs of the pid position controller
    float outputp[4];
    /// @brief outputs of the pid velocity controller
    float outputv[4];
    /// @brief combined outputs of the pid position and velocity controllers
    float output[4];
    /// @brief target motor velocity
    float motor_velocity[4];
public:
    /// @brief default
    XDriveVelocityController() {
        xy_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::XYVelocityController);
        chassis_angle_controller = controller_config.get_sub_controller_by_type(NewConfig::ChassisAngleController);
        chassis_angular_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::ChassisAngularVelocityController);
        low_level_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::LowLevelVelocityController);
        power_buffer_controller = controller_config.get_sub_controller_by_type(NewConfig::PowerBufferController);
    }

    /// @brief take s in a micro_reference of wheel velocity
    /// @param reference current target robot state
    /// @param estimate current robot state estimate
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs current outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);
    // New overload using RobotStateMap
    void step(RobotStateMap reference_map, RobotStateMap estimate_map, float outputs[CAN_MAX_MOTORS]);

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        for (int i = 0; i < 3; i++) {
            pidp[i].sumError = 0.0;
            pidv[i].sumError = 0.0;
        }
    }
};

/// @brief Controller for the yaw
struct YawController : public Controller {
private:
    NewConfig::SubController full_state_position_controller;
    NewConfig::SubController full_state_velocity_controller;

    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;

public:
    /// @brief default constructor
    YawController() {
        full_state_position_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStatePositionController);
        full_state_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStateVelocityController);
    }

    /// @brief calculate motor outputs based on reference and estimate
    /// @param reference current target robot state
    /// @param estimate current estimate robot state
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs motor outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);
    // New overload using RobotStateMap
    void step(RobotStateMap reference_map, RobotStateMap estimate_map, float outputs[CAN_MAX_MOTORS]);

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        pidp.sumError = 0.0;
        pidv.sumError = 0.0;
    }
};
/// @brief Controller for the pitch
struct PitchController : public Controller {

private:
    NewConfig::SubController full_state_position_controller;
    NewConfig::SubController full_state_velocity_controller;

    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;

public:
    PitchController() {
        full_state_position_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStatePositionController);
        full_state_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStateVelocityController);
    }
    /// @brief calculate motor outputs based on reference and estimate
    /// @param reference current target robot state
    /// @param estimate current estimate robot state
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs motor outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);
    // New overload using RobotStateMap
    void step(RobotStateMap reference_map, RobotStateMap estimate_map, float outputs[CAN_MAX_MOTORS]);

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        pidp.sumError = 0.0;
        pidv.sumError = 0.0;
    }
};

/// @brief Controller for the flywheels
struct FlywheelController : public Controller {
private:
    NewConfig::SubController high_level_velocity_controller;
    NewConfig::SubController low_level_velocity_controller;

    /// @brief filter for controlling meters/second
    PIDFilter pid_high;
    /// @brief filter for controlling motor velocity
    PIDFilter pid_low;
public:
    /// @brief default constructor
    FlywheelController() {
        high_level_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::HighLevelVelocityController);
        low_level_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::LowLevelVelocityController);
    }

    /// @brief calculate motor outputs based on reference and estimate
    /// @param reference current target robot state
    /// @param estimate current estimate robot state
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs motor outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        pid_high.sumError = 0.0;
        pid_low.sumError = 0.0;
    }
};

// /// @brief Controls the feeder
// struct FeederController : public Controller {
// private:
//     NewConfig::SubController full_state_position_controller;
//     NewConfig::SubController full_state_velocity_controller;
//     /// @brief filter for controlling balls/sec
//     PIDFilter pid_high;
//     /// @brief filter for controlling motor velocity
//     PIDFilter pid_low;

// public:
//     /// @brief default constructor
//     FeederController() {
//         full_state_position_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStatePositionController);
//         full_state_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStateVelocityController);
//     }

//     /// @brief calculate motor outputs based on reference and estimate
//     /// @param reference current target robot state
//     /// @param estimate current estimate robot state
//     /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
//     /// @param outputs motor outputs
//     void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);

//     /// @brief reset the controller
//     inline void reset() {
//         Controller::reset();
//         pid_high.sumError = 0.0;
//         pid_low.sumError = 0.0;
//     }
// };

/// @brief Controller for the ball feeder
struct FeederController : public Controller {
    private:
        NewConfig::SubController full_state_position_controller;
        NewConfig::SubController full_state_velocity_controller;
        /// @brief filter for calculating pid position controller outputs
        PIDFilter pidp;
        /// @brief filter for calculating pid velocity controller outputs
        PIDFilter pidv;
    public:
        /// @brief assign sub controllers
        FeederController() {
            full_state_position_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStatePositionController);
            full_state_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStateVelocityController);
        }
        /// @brief calculate motor outputs based on reference and estimate
        /// @param reference current target robot state
        /// @param estimate current estimate robot state
        /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
        /// @param outputs motor outputs
        void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);
        // New overload using RobotStateMap
        void step(RobotStateMap reference_map, RobotStateMap estimate_map, float outputs[CAN_MAX_MOTORS]);
    
        /// @brief reset the controller
        inline void reset() {
            Controller::reset();
            pidp.sumError = 0.0;
            pidv.sumError = 0.0;
        }
    };

#endif // CONTROLLER_H