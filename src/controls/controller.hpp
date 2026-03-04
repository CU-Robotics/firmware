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

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    void step(RobotStateMap reference_map, RobotStateMap estimate_map);

    /// @brief Resets integrators/timers
    virtual void reset() { timer.start(); }

    const NewConfig::Controller& config() const { return controller_config; }
};

/// @brief Position controller for the chassis
struct XDriveController : public Controller {
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
    
    Motor* chassis_motor_1; // front left
    Motor* chassis_motor_2; // front right
    Motor* chassis_motor_3; // back right
    Motor* chassis_motor_4; // back left

    const StateName& chassis_x_state;
    const StateName& chassis_y_state;
    const StateName& chassis_heading_state;

public:
    /// @brief default
    XDrivePositionController(const NewConfig::Controller& controller_config, CANManager& can) : 
        chassis_x_state(controller_config.chassis_x_state), chassis_y_state(controller_config.chassis_y_state), 
        chassis_heading_state(controller_config.chassis_heading_state), Controller(controller_config) {

        xy_position_controller = controller_config.get_sub_controller_by_type(NewConfig::XYPositionController);
        xy_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::XYVelocityController);
        chassis_angle_controller = controller_config.get_sub_controller_by_type(NewConfig::ChassisAngleController);
        chassis_angular_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::ChassisAngularVelocityController);
        low_level_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::LowLevelVelocityController);
        power_buffer_controller = controller_config.get_sub_controller_by_type(NewConfig::PowerBufferController);

        chassis_motor_1 = can.get_motor_by_name(controller_config.motors.chassis_motor_1);
        chassis_motor_2 = can.get_motor_by_name(controller_config.motors.chassis_motor_2);
        chassis_motor_3 = can.get_motor_by_name(controller_config.motors.chassis_motor_3);
        chassis_motor_4 = can.get_motor_by_name(controller_config.motors.chassis_motor_4);
    }

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    void step(RobotStateMap reference_map, RobotStateMap estimate_map);

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

    Motor* yaw_motor_1;
    Motor* yaw_motor_2;

    const StateName& yaw_angle_state;

public:
    /// @brief default constructor
    YawController(const NewConfig::Controller& controller_config, CANManager& can) : yaw_angle_state(controller_config.yaw_angle_state), Controller(controller_config) {
        full_state_position_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStatePositionController);
        full_state_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStateVelocityController);

        yaw_motor_1 = can.get_motor_by_name(controller_config.motors.yaw_motor_1);
        yaw_motor_2 = can.get_motor_by_name(controller_config.motors.yaw_motor_2);
    }

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    void step(RobotStateMap reference_map, RobotStateMap estimate_map);

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

    Motor* pitch_motor_1;
    Motor* pitch_motor_2;

    const StateName& pitch_angle_state;
public:
    PitchController(const NewConfig::Controller& controller_config, CANManager& can) : Controller(controller_config), pitch_angle_state(controller_config.pitch_angle_state) {
        full_state_position_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStatePositionController);
        full_state_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStateVelocityController);

        pitch_motor_1 = can.get_motor_by_name(controller_config.motors.pitch_motor_1);
        pitch_motor_2 = can.get_motor_by_name(controller_config.motors.pitch_motor_2);
    }

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    void step(RobotStateMap reference_map, RobotStateMap estimate_map);

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

    Motor* flywheel_motor_1;
    Motor* flywheel_motor_2;

    const StateName& flywheel_velocity_state;
public:
    /// @brief default constructor
    FlywheelController(const NewConfig::Controller& controller_config, CANManager& can) : Controller(controller_config), flywheel_velocity_state(controller_config.flywheel_velocity_state) {
        high_level_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::HighLevelVelocityController);
        low_level_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::LowLevelVelocityController);

        flywheel_motor_1 = can.get_motor_by_name(controller_config.motors.flywheel_motor_1);
        flywheel_motor_2 = can.get_motor_by_name(controller_config.motors.flywheel_motor_2);
    }

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    void step(RobotStateMap reference_map, RobotStateMap estimate_map);

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

        Motor* feeder_motor;

        const StateName& feeder_velocity_state;
    public:
        /// @brief assign sub controllers
        FeederController() : Controller(controller_config), feeder_velocity_state(controller_config.feeder_velocity_state) {
            full_state_position_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStatePositionController);
            full_state_velocity_controller = controller_config.get_sub_controller_by_type(NewConfig::FullStateVelocityController);

            feeder_motor = can.get_motor_by_name(controller_config.motors.feeder_motor);
        }

        /// @brief sends motor commands based on a reference and estimated state
        /// @param reference_map current target robot state map
        /// @param estimate_map current estimate robot state map
        void step(RobotStateMap reference_map, RobotStateMap estimate_map);
    
        /// @brief reset the controller
        inline void reset() {
            Controller::reset();
            pidp.sumError = 0.0;
            pidv.sumError = 0.0;
        }
    };

#endif // CONTROLLER_H