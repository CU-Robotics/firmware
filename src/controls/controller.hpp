#pragma once

#include "filters/pid_filter.hpp"
#include "motor.hpp"
#include "safety.hpp"
#include "utils/timing.hpp"
#include "sensors/can/can_manager.hpp"
#include "robot_state_map.hpp"

#include "comms/config_data/controller.hpp"
#include <memory>

/// @brief Parent controller struct, all controllers should be based off of this.
struct Controller {
protected:
    /// @brief config data for this specific controller
    const Cfg::Controller& controller_config;

    /// @brief Timer object so we can use dt in controllers
    Timer timer;
    
public:
    /// @brief Construct the controller and get the config data
    /// @param _controller_config config data for this controller
    Controller(const Cfg::Controller& _controller_config) : controller_config(_controller_config) { };

    virtual ~Controller() { };

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    virtual void step(RobotStateMap& reference_map, RobotStateMap& estimate_map) = 0;

    /// @brief Resets integrators/timers
    virtual void reset() { timer.start(); }
    /// @brief Get the controller configuration data
    /// @return const reference to the controller config struct for this controller
    const Cfg::Controller& config() const { return controller_config; }
    /// @brief Helper function to get a motor by its generic use. Will trigger safety procedure if the motor is not available.
    /// @param use the generic use of the motor to get
    /// @param can reference to the CAN manager to get motor objects so we can write directly to motors
    /// @param available_motors list of motor names that are available to be used; this is to prevent multiple controllers from trying to control the same motor
    /// @return shared pointer to the motor object that corresponds to the generic use requested
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
    /// @brief control chassis position in x and y axis
    Cfg::SubController xy_position_controller;
    /// @brief control chassis velocity in x and y axis
    Cfg::SubController xy_velocity_controller;
    /// @brief control chassis angle
    Cfg::SubController chassis_angle_controller;
    /// @brief control chassis angular velocity
    Cfg::SubController chassis_angular_velocity_controller;
    /// @brief control the actual motor velocities based on the outputs of the higher level controllers
    Cfg::SubController low_level_velocity_controller;
    /// @brief control input to motors based on ref power buffer so we don't draw too much.
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

    /// @brief front left chassis motor
    std::shared_ptr<Motor> chassis_motor_1; // front left
    /// @brief front right chassis motor
    std::shared_ptr<Motor> chassis_motor_2; // front right
    /// @brief back right chassis motor
    std::shared_ptr<Motor> chassis_motor_3; // back right
    /// @brief back left chassis motor
    std::shared_ptr<Motor> chassis_motor_4; // back left

    /// @brief state name for the chassis x axis
    const Cfg::StateName& chassis_x_state;
    /// @brief state name for the chassis y axis
    const Cfg::StateName& chassis_y_state;
    /// @brief state name for the chassis yaw axis
    const Cfg::StateName& chassis_heading_state;

public:
    /// @brief Construct the controller and get the subcontroller configs and motor objects from the config data
    /// @param controller_config config data for this controller
    /// @param can reference to the CAN manager to get motor objects so we can write directly to motors
    /// @param available_motors list of motor names that are available to be used; this is to prevent multiple controllers from trying to control the same motor
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

        chassis_motor_1 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::ChassisFrontRight, can, available_motors);
        chassis_motor_2 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::ChassisBackRight, can, available_motors);
        chassis_motor_3 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::ChassisBackLeft, can, available_motors);
        chassis_motor_4 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::ChassisFrontLeft, can, available_motors);
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
    /// @brief control yaw position
    Cfg::SubController full_state_position_controller;
    /// @brief control yaw velocity
    Cfg::SubController full_state_velocity_controller;

    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;
    /// @brief yaw motor 1
    std::shared_ptr<Motor> yaw_motor_1;
    /// @brief yaw motor 2
    std::shared_ptr<Motor> yaw_motor_2;
    /// @brief state name for the yaw axis
    const Cfg::StateName& yaw_angle_state;
public:
    /// @brief Construct the controller and get the subcontroller configs and motor objects from the config data
    /// @param controller_config config data for this controller
    /// @param can reference to the CAN manager to get motor objects so we can write directly
    /// @param available_motors list of motor names that are available to be used; this is to prevent multiple controllers from trying to control the same motor
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
    /// @brief control position in the pitch axis
    Cfg::SubController full_state_position_controller;
    /// @brief control velocity in the pitch axis
    Cfg::SubController full_state_velocity_controller;
    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;
    /// @brief pitch motor 1
    std::shared_ptr<Motor> pitch_motor_1;
    /// @brief pitch motor 2
    std::shared_ptr<Motor> pitch_motor_2;
    /// @brief state name for the pitch axis
    const Cfg::StateName& pitch_angle_state;
public:
    /// @brief Construct the controller and get the subcontroller configs and motor objects from the config data
    /// @param controller_config config data for this controller
    /// @param can reference to the CAN manager to get motor objects so we can write directly to motors
    /// @param available_motors list of motor names that are available to be used; this is to prevent multiple controllers from trying to control the same motor
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
    /// @brief control velocity of the balls launched by the flywheels
    Cfg::SubController high_level_velocity_controller;
    /// @brief control the actual motor velocities based on the outputs of the higher level controller
    Cfg::SubController low_level_velocity_controller;

    /// @brief filter for controlling meters/second
    PIDFilter pid_high;
    /// @brief filter for controlling motor velocity
    PIDFilter pid_low;

    /// @brief flywheel motor 1
    std::shared_ptr<Motor> flywheel_motor_1;
    /// @brief flywheel motor 2
    std::shared_ptr<Motor> flywheel_motor_2;

    /// @brief state name for the flywheel velocity
    const Cfg::StateName& flywheel_velocity_state;
public:
    /// @brief Construct the controller and get the subcontroller configs and motor objects from the config data
    /// @param controller_config config data for this controller
    /// @param can reference to the CAN manager to get motor objects so we can write directly
    /// @param available_motors list of motor names that are available to be used; this is to prevent multiple controllers from trying to control the same motor
    FlywheelController(const Cfg::Controller& controller_config, CANManager& can, std::vector<Cfg::MotorName>& available_motors) : Controller(controller_config), flywheel_velocity_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::ShooterBallVelocity)) {
        high_level_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::HighLevelVelocityController);
        low_level_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::LowLevelVelocityController);

        flywheel_motor_1 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::FlywheelLeft, can, available_motors);
        flywheel_motor_2 = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::FlywheelRight, can, available_motors);
    }

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
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
        /// @brief control position of the feeder
        Cfg::SubController full_state_position_controller;
        /// @brief control velocity of the feeder
        Cfg::SubController full_state_velocity_controller;
        /// @brief filter for calculating pid position controller outputs
        PIDFilter pidp;
        /// @brief filter for calculating pid velocity controller outputs
        PIDFilter pidv;
        /// @brief motor attached to the feeder
        std::shared_ptr<Motor> feeder_motor;
        /// @brief state name for the feeder position
        const Cfg::StateName& feeder_position_state;
    public:
        /// @brief Construct the controller and get the subcontroller configs and motor objects from the config data
        /// @param controller_config config data for this controller
        /// @param can reference to the CAN manager to get motor objects so we can write directly to motors
        /// @param available_motors list of motor names that are available to be used; this is to prevent multiple controllers from trying to control the same motor
        FeederController(const Cfg::Controller& controller_config, CANManager& can, std::vector<Cfg::MotorName>& available_motors) : Controller(controller_config),
            feeder_position_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::FeederBallPosition)) {
            full_state_position_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::FullStatePositionController);
            full_state_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::FullStateVelocityController);
                
            feeder_motor = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::Feeder, can, available_motors);
        }

        /// @brief sends motor commands based on a reference and estimated state
        /// @param reference_map current target robot state map
        /// @param estimate_map current estimate robot state map
        void step(RobotStateMap& reference_map, RobotStateMap& estimate_map);
    
        /// @brief reset the controller
        inline void reset() {
            Controller::reset();
            pidp.sumError = 0.0;
            pidv.sumError = 0.0;
        }
};