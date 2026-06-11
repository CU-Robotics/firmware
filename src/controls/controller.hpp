#pragma once

#include "estimator.hpp"
#include "filters/pid_filter.hpp"
#include "motor.hpp"
#include "safety.hpp"
#include "utils/timing.hpp"
#include "sensors/can/can_manager.hpp"
#include "robot_state_map.hpp"
#include "reference_governor.hpp"

#include "comms/config_data/controller.hpp"
#include <memory>
#include <cmath>

/// @brief Parent controller struct, all controllers should be based off of this.
struct Controller {
protected:
    /// @brief config data for this specific controller
    const Cfg::Controller& controller_config;

    /// @brief Timer object so we can use dt in controllers
    Timer timer;

    /// @brief Internal tracking for controller error exceedance on a single state.
    struct ErrorMonitor {
        /// @brief Whether the error monitor has been initialized with an initial error value
        bool initialized = false;
        /// @brief Whether the error is currently exceeding the configured limit
        bool exceeding = false;
        /// @brief The previous control loop error value, used for unwrapping wrapped errors
        float previous_error = 0.0f;
        /// @brief Timestamp in microseconds when the error first exceeded the limit
        uint32_t exceed_start_us = 0;
    };
    
public:
    /// @brief Construct the controller and get the config data
    /// @param _controller_config config data for this controller
    Controller(const Cfg::Controller& _controller_config) : controller_config(_controller_config) { };

    /// @brief Virtual destructor since this is a parent class
    virtual ~Controller() { };

    /// @brief sends motor commands based on a reference and estimated state
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    /// @param target_map current target robot state map
    virtual void step(RobotStateMap& reference_map, RobotStateMap& estimate_map, RobotStateMap& target_map) = 0;

    /// @brief Validate controller state before stepping.
    /// Managers call this so controller-specific checks live outside the control loop itself.
    /// @param reference_map current target robot state map
    /// @param estimate_map current estimate robot state map
    virtual void validate(const RobotStateMap& reference_map, const RobotStateMap& estimate_map) { }

    /// @brief Resets integrators/timers
    virtual void reset() { timer.start(); }
    /// @brief Get the controller configuration data
    /// @return const reference to the controller config struct for this controller
    const Cfg::Controller& config() const { return controller_config; }

protected:
    /// @brief Handle a controller-specific error event after the shared exceedance check has decided to escalate.
    /// Default behavior is to call the global safety procedure; controllers may override to implement custom handling.
    /// @param controller_name Name of the controller for diagnostics.
    /// @param state_name Name of the controlled state for diagnostics.
    /// @param reference_state Target state.
    /// @param estimate_state Estimated state.
    /// @param error The signed controller error that triggered the escalation.
    virtual void handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error);

    /// @brief Get the state value used for controller error checks based on the state governor type.
    /// @param state The state to inspect.
    /// @return The state value corresponding to the configured governor type.
    static float get_state_error_value(const State& state) {
        switch (state.config().governor_type) {
            case Cfg::StateOrder::Position:
                return state.get_position();
            case Cfg::StateOrder::Velocity:
                return state.get_velocity();
            case Cfg::StateOrder::Acceleration:
                return state.get_acceleration();
            default:
                return state.get_position();
        }
    }

    /// @brief Check a single state pair for error exceedance and dispatch to the controller-specific error handler if the exceedance persists too long.
    /// @param controller_name Name of the controller for diagnostics.
    /// @param state_name Name of the controlled state for diagnostics.
    /// @param reference_state Target state.
    /// @param estimate_state Estimated state.
    /// @param monitor Persistent monitoring state for this controlled state.
    void checkControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, ErrorMonitor& monitor);

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

    /// @brief error tracking for the chassis x axis
    ErrorMonitor chassis_x_error_monitor;
    /// @brief error tracking for the chassis y axis
    ErrorMonitor chassis_y_error_monitor;
    /// @brief error tracking for the chassis heading axis
    ErrorMonitor chassis_heading_error_monitor;

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
    /// @param target_map current target robot state map
    void step(RobotStateMap& reference_map, RobotStateMap& estimate_map, RobotStateMap& target_map);

    /// @copydoc Controller::validate
    void validate(const RobotStateMap& reference_map, const RobotStateMap& estimate_map) override;

    /// @brief Handle a chassis controller error.
    /// @param controller_name Name of the controller for diagnostics.
    /// @param state_name Name of the controlled state for diagnostics.
    /// @param reference_state Target state.
    /// @param estimate_state Estimated state.
    /// @param error The signed controller error that triggered the escalation.
    void handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) override;

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        for (int i = 0; i < 3; i++) {
            pidp[i].sumError = 0.0;
            pidv[i].sumError = 0.0;
        }
        chassis_x_error_monitor = ErrorMonitor{};
        chassis_y_error_monitor = ErrorMonitor{};
        chassis_heading_error_monitor = ErrorMonitor{};
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

    /// @brief error tracking for the yaw axis
    ErrorMonitor yaw_error_monitor;
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
    /// @param target_map current target robot state map
    void step(RobotStateMap& reference_map, RobotStateMap& estimate_map, RobotStateMap& target_map);

    /// @copydoc Controller::validate
    void validate(const RobotStateMap& reference_map, const RobotStateMap& estimate_map) override;

    /// @brief Handle a yaw controller error.
    /// @param controller_name Name of the controller for diagnostics.
    /// @param state_name Name of the controlled state for diagnostics.
    /// @param reference_state Target state.
    /// @param estimate_state Estimated state.
    /// @param error The signed controller error that triggered the escalation.
    void handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) override;

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        pidp.sumError = 0.0;
        pidv.sumError = 0.0;
        yaw_error_monitor = ErrorMonitor{};
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

    /// @brief error tracking for the pitch axis
    ErrorMonitor pitch_error_monitor;
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
    /// @param target_map current target robot state map
    void step(RobotStateMap& reference_map, RobotStateMap& estimate_map, RobotStateMap& target_map);

    /// @copydoc Controller::validate
    void validate(const RobotStateMap& reference_map, const RobotStateMap& estimate_map) override;

    /// @brief Handle a pitch controller error.
    /// @param controller_name Name of the controller for diagnostics.
    /// @param state_name Name of the controlled state for diagnostics.
    /// @param reference_state Target state.
    /// @param estimate_state Estimated state.
    /// @param error The signed controller error that triggered the escalation.
    void handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) override;

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        pidp.sumError = 0.0;
        pidv.sumError = 0.0;
        pitch_error_monitor = ErrorMonitor{};
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

    /// @brief error tracking for the flywheel state
    ErrorMonitor flywheel_error_monitor;
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
    /// @param target_map current target robot state map
    void step(RobotStateMap& reference_map, RobotStateMap& estimate_map, RobotStateMap& target_map);

    /// @copydoc Controller::validate
    void validate(const RobotStateMap& reference_map, const RobotStateMap& estimate_map) override;

    /// @brief Handle a flywheel controller error.
    /// @param controller_name Name of the controller for diagnostics.
    /// @param state_name Name of the controlled state for diagnostics.
    /// @param reference_state Target state.
    /// @param estimate_state Estimated state.
    /// @param error The signed controller error that triggered the escalation.
    void handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) override;

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        pid_high.sumError = 0.0;
        pid_low.sumError = 0.0;
        flywheel_error_monitor = ErrorMonitor{};
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

        /// @brief error tracking for the feeder state
        ErrorMonitor feeder_error_monitor;
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
        /// @param target_map current target robot state map
        void step(RobotStateMap& reference_map, RobotStateMap& estimate_map, RobotStateMap& target_map);

        /// @copydoc Controller::validate
        void validate(const RobotStateMap& reference_map, const RobotStateMap& estimate_map) override;

        /// @brief Handle a feeder controller error.
        /// @param controller_name Name of the controller for diagnostics.
        /// @param state_name Name of the controlled state for diagnostics.
        /// @param reference_state Target state.
        /// @param estimate_state Estimated state.
        /// @param error The signed controller error that triggered the escalation.
        void handleControllerError(const char* controller_name, const char* state_name, const State& reference_state, const State& estimate_state, float error) override;
    
        /// @brief reset the controller
        inline void reset() {
            Controller::reset();
            pidp.sumError = 0.0;
            pidv.sumError = 0.0;
            feeder_error_monitor = ErrorMonitor{};
        }
    };

/// @brief Controller for the lower ball feeder on bottom fed
struct LowerFeederController : public Controller {
    private:
        /// @brief control position of the upper feeder
        Cfg::SubController upper_position_controller;
        /// @brief control velocity of the upper feeder
        Cfg::SubController upper_velocity_controller;
        /// @brief control position of the lower feeder
        Cfg::SubController lower_position_controller;
        /// @brief control velocity of the lower feeder
        Cfg::SubController lower_velocity_controller;
        /// @brief filter for calculating pid position controller outputs
        PIDFilter upper_pidp;
        /// @brief filter for calculating pid velocity controller outputs
        PIDFilter upper_pidv;
        /// @brief filter for calculating pid position controller outputs
        PIDFilter lower_pidp;
        /// @brief filter for calculating pid velocity controller outputs
        PIDFilter lower_pidv;
        /// @brief motor attached to the feeder
        std::shared_ptr<Motor> near_feeder_motor;
        /// @brief motor attached to the feeder
        std::shared_ptr<Motor> far_feeder_motor;
        /// @brief motor attached to the upper feeder
        std::shared_ptr<Motor> upper_feeder_motor;
        /// @brief state name for the upper feeder position
        const Cfg::StateName& upper_feeder_position_state;
        /// @brief state name for the lower feeder position
        const Cfg::StateName& lower_feeder_position_state;

        /// @brief error tracking for the lower feeder state
        ErrorMonitor lower_feeder_error_monitor;

        /// @brief A reference kept internal to the controller which can update the upper feeder position independently of the main reference map.
        RobotStateMap upper_feeder_reference_state;
        /// @brief Governor for the upper feeder reference state
        Governor upper_feeder_reference_governor;
        /// @brief for internally tracking the target position of the upper feeder
        RobotStateMap upper_target;

        float target_increase_time = 0.0;

        bool timer_active = false;
    public:
        /// @brief Construct the controller and get the subcontroller configs and motor objects from the config data
        /// @param controller_config config data for this controller
        /// @param can reference to the CAN manager to get motor objects so we can write directly to motors
        /// @param available_motors list of motor names that are available to be used; this is to prevent multiple controllers from trying to control the same motor
        LowerFeederController(const Cfg::Controller& controller_config, CANManager& can, std::vector<Cfg::MotorName>& available_motors, const std::vector<Cfg::State>& state_config) : Controller(controller_config),
            upper_feeder_position_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::UpperFeederBallPosition)),
            lower_feeder_position_state(controller_config.get_state_name_by_generic_use(Cfg::GenericControllerStateUse::LowerFeederBallPosition)),
            upper_feeder_reference_state({}), upper_feeder_reference_governor({}), upper_target({}) {
            lower_position_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::LowerFeederPositionController);
            lower_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::LowerFeederVelocityController);
            upper_position_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::UpperFeederPositionController);
            upper_velocity_controller = controller_config.get_sub_controller_by_type(Cfg::SubControllerType::UpperFeederVelocityController);

            near_feeder_motor = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::NearFeeder, can, available_motors);
            far_feeder_motor = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::FarFeeder, can, available_motors);
            upper_feeder_motor = get_motor_by_generic_use(Cfg::GenericControllerMotorUse::UpperFeeder, can, available_motors);

            bool found = false;
            for (auto& state: state_config) {
                if (state.name == upper_feeder_position_state) {
                    Serial.printf("state config, reference limits velocity: min %f, max %f\n", state.reference_limits.velocity.min, state.reference_limits.velocity.max);
                    upper_feeder_reference_state.get_state_map().emplace(upper_feeder_position_state, State(state));
                    upper_target.get_state_map().emplace(upper_feeder_position_state, State(state));
                    std::vector<Cfg::State> state_config_vec = {};
                    state_config_vec.push_back(state);
                    upper_feeder_reference_governor = Governor(state_config_vec);
                    upper_feeder_reference_governor.set_reference_map(upper_feeder_reference_state);
                    found = true;
                }
            }
            if (!found) {
                safety::safety_procedure("Upper Feeder state config not found in config");
            }
        }

        /// @brief sends motor commands based on a reference and estimated state
        /// @param reference_map current target robot state map
        /// @param estimate_map current estimate robot state map
        /// @param target_map current target robot state map
        void step(RobotStateMap& reference_map, RobotStateMap& estimate_map, RobotStateMap& target_map);

        /// @copydoc Controller::validate
        void validate(const RobotStateMap& reference_map, const RobotStateMap& estimate_map) override;
    
        /// @brief reset the controller
        inline void reset() {
            Controller::reset();
            upper_pidp.sumError = 0.0;
            upper_pidv.sumError = 0.0;
            lower_pidp.sumError = 0.0;
            lower_pidv.sumError = 0.0;
            lower_feeder_error_monitor = ErrorMonitor{};
        }
};