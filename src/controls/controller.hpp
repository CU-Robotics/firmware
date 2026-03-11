#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "estimator.hpp"
#include "filters/pid_filter.hpp"
#include "motor.hpp"
#include "safety.hpp"
#include "utils/timing.hpp"
#include "sensors/can/can_manager.hpp"
#include "robot_state_map.hpp"

#include "comms/config_data/controller.hpp"
#include <memory>

#define NUM_GAINS 24
#define NUM_ROBOT_CONTROLLERS 12

/// @brief Parent controller struct, all controllers should be based off of this.
struct Controller {
protected:
    /// @brief config data for this specific controller
    const Cfg::Controller& controller_config;

    /// @brief Timer object so we can use dt in controllers
    Timer timer;
    
public:
    /// @brief default constructor
    Controller(const Cfg::Controller& _controller_config) : controller_config(_controller_config) { };

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    /// @param output_data data structure to store the controller outputs for sending to comms
    virtual void step(RobotStateMap& reference_map, RobotStateMap& estimate_map) = 0;

    /// @brief Resets integrators/timers
    virtual void reset() { timer.start(); }

    const Cfg::Controller& config() const { return controller_config; }

    std::shared_ptr<Motor> get_motor_by_generic_use(Cfg::GenericControllerMotorUse use, CANManager& can, std::vector<Cfg::MotorName>& available_motors) const {
        Cfg::MotorName requested_motor_name = controller_config.get_motor_name_by_generic_use(use);
        for (const auto& motor_name : available_motors) {
            if (motor_name == requested_motor_name) { 
                return can.get_motor_by_name(requested_motor_name);
            }        
        }
        safety::safety_procedure("Controller requires motor that is not available. (multiple controllers may be trying to use the same motor)");
        
    }
};

/// @brief Position controller for the chassis
struct XDriveController : public Controller {
private:
    Cfg::SubController xy_position_controller;
    Cfg::SubController xy_velocity_controller;
    Cfg::SubController chassis_angle_controller;
    Cfg::SubController chassis_angular_velocity_controller;
    Cfg::SubController low_level_velocity_controller;
    Cfg::SubController power_buffer_controller;

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
    
    std::shared_ptr<Motor> chassis_motor_1; // front left
    std::shared_ptr<Motor> chassis_motor_2; // front right
    std::shared_ptr<Motor> chassis_motor_3; // back right
    std::shared_ptr<Motor> chassis_motor_4; // back left

    const Cfg::StateName& chassis_x_state;
    const Cfg::StateName& chassis_y_state;
    const Cfg::StateName& chassis_heading_state;

public:
    /// @brief default
    XDriveController(const Cfg::Controller& controller_config, CANManager& can, std::vector<Cfg::MotorName>& available_motors) : 
        Controller(controller_config), chassis_x_state{controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::ChassisX)}, 
        chassis_y_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::ChassisY)), 
        chassis_heading_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::ChassisHeading)) {

        xy_position_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::XYPositionController);
        xy_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::XYVelocityController);
        chassis_angle_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::ChassisAngleController);
        chassis_angular_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::ChassisAngularVelocityController);
        low_level_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::LowLevelVelocityController);
        power_buffer_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::PowerBufferController);

        chassis_motor_1 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::ChassisFrontLeft, can, available_motors);
        chassis_motor_2 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::ChassisBackRight, can, available_motors);
        chassis_motor_3 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::ChassisBackLeft, can, available_motors);
        chassis_motor_4 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::ChassisFrontRight, can, available_motors);
    }

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    void step(RobotStateMap& reference_map, RobotStateMap& estimate_map);

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
    Cfg::SubController full_state_position_controller;
    Cfg::SubController full_state_velocity_controller;

    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;

    std::shared_ptr<Motor> yaw_motor_1;
    std::shared_ptr<Motor> yaw_motor_2;

    const Cfg::StateName& yaw_angle_state;

public:
    /// @brief default constructor
    YawController(const Cfg::Controller& controller_config, CANManager& can, std::vector<Cfg::MotorName>& available_motors) : Controller(controller_config),
        yaw_angle_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::GimbalYaw)) {
        full_state_position_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::FullStatePositionController);
        full_state_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::FullStateVelocityController);

        yaw_motor_1 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::Yaw1, can, available_motors);
        yaw_motor_2 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::Yaw2, can, available_motors);
    }

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    /// @param output_data data structure to store the controller outputs for sending to comms
    void step(RobotStateMap& reference_map, RobotStateMap& estimate_map);

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
    Cfg::SubController full_state_position_controller;
    Cfg::SubController full_state_velocity_controller;

    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;

    std::shared_ptr<Motor> pitch_motor_1;
    std::shared_ptr<Motor> pitch_motor_2;

    const Cfg::StateName& pitch_angle_state;
public:
    PitchController(const Cfg::Controller& controller_config, CANManager& can, std::vector<Cfg::MotorName>& available_motors) : Controller(controller_config),
        pitch_angle_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::GimbalPitch)) {
        full_state_position_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::FullStatePositionController);
        full_state_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::FullStateVelocityController);

        pitch_motor_1 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::PitchLeft, can, available_motors);
        pitch_motor_2 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::PitchRight, can, available_motors);
    }

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    /// @param output_data data structure to store the controller outputs for sending to comms
    void step(RobotStateMap& reference_map, RobotStateMap& estimate_map);

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
    Cfg::SubController high_level_velocity_controller;
    Cfg::SubController low_level_velocity_controller;

    /// @brief filter for controlling meters/second
    PIDFilter pid_high;
    /// @brief filter for controlling motor velocity
    PIDFilter pid_low;

    std::shared_ptr<Motor> flywheel_motor_1;
    std::shared_ptr<Motor> flywheel_motor_2;

    const Cfg::StateName& flywheel_velocity_state;
public:
    /// @brief default constructor
    FlywheelController(const Cfg::Controller& controller_config, CANManager& can, std::vector<Cfg::MotorName>& available_motors) : Controller(controller_config), flywheel_velocity_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::ShooterBallVelocity)) {
        high_level_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::HighLevelVelocityController);
        low_level_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::LowLevelVelocityController);

        flywheel_motor_1 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::FlywheelLeft, can, available_motors);
        flywheel_motor_2 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::FlywheelRight, can, available_motors);
    }

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    /// @param output_data data structure to store the controller outputs for sending to comms
    void step(RobotStateMap& reference_map, RobotStateMap& estimate_map);

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        pid_high.sumError = 0.0;
        pid_low.sumError = 0.0;
    }
};

/// @brief Controller for the ball feeder
struct FeederController : public Controller {
    private:
        Cfg::SubController full_state_position_controller;
        Cfg::SubController full_state_velocity_controller;
        /// @brief filter for calculating pid position controller outputs
        PIDFilter pidp;
        /// @brief filter for calculating pid velocity controller outputs
        PIDFilter pidv;

        std::shared_ptr<Motor> feeder_motor;

        const Cfg::StateName& feeder_position_state;
    public:
        /// @brief assign sub controllers
        FeederController(const Cfg::Controller& controller_config, CANManager& can, std::vector<Cfg::MotorName>& available_motors) : Controller(controller_config),
            feeder_position_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::FeederBallPosition)) {
            full_state_position_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::FullStatePositionController);
            full_state_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::FullStateVelocityController);
                
            feeder_motor = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::Feeder, can, available_motors);
        }

        /// @brief sends motor commands based on a reference and estimated state
        /// @param reference_map current target robot state map
        /// @param estimate_map current estimate robot state map
        /// @param output_data data structure to store the controller outputs for sending to comms
        void step(RobotStateMap& reference_map, RobotStateMap& estimate_map);
    
        /// @brief reset the controller
        inline void reset() {
            Controller::reset();
            pidp.sumError = 0.0;
            pidv.sumError = 0.0;
        }
    };

#endif // CONTROLLER_H