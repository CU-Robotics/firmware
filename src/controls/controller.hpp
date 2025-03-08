#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "filters/pid_filter.hpp"
#include "utils/timing.hpp"
#include "sensors/can/can_manager.hpp"
#include "sensors/RefSystem.hpp"
#include "state.hpp"
#include "sensors/ACS712.hpp"

#define NUM_GAINS 24
#define NUM_ROBOT_CONTROLLERS 12

/// @brief Parent controller struct, all controllers should be based off of this.
struct Controller {
protected:
    /// @brief Gains for this specific controller. Each controller has a unique set of gains that mean different things. Refer to the config yaml for more in depth gains info.
    float gains[NUM_GAINS];
    /// @brief list of numbers to help relate motors to joints so we can take in an error in joint space and output motor current. Refer to config yaml for more info.
    float gear_ratios[CAN_MAX_MOTORS];
    /// @brief Timer object so we can use dt in controllers
    Timer timer;

public:
    /// @brief default constructor
    Controller() { };

    /// @brief set the gains for this specific controller
    /// @param _gains gains array of length NUM_GAINS
    inline void set_gains(const float _gains[NUM_GAINS]) {
        for (int i = 0; i < NUM_GAINS; i++) {
            gains[i] = _gains[i];
        }
    }

    /// @brief set the gear ratios for this specific controller
    /// @param _gear_ratios gains array of length NUM_MOTORS
    inline void set_gear_ratios(const float _gear_ratios[CAN_MAX_MOTORS]) {
        for (size_t i = 0; i < CAN_MAX_MOTORS; i++) gear_ratios[i] = _gear_ratios[i];
    }

    /// @brief Generates an output from a state reference and estimation
    /// @param reference reference (target state)
    /// @param estimate current macro state estimate
    /// @param micro_estimate current micro state estimate
    /// @param outputs array of new motor inputs
    virtual void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) {}

    /// @brief Resets integrators/timers
    virtual void reset() { timer.start(); }
};

/// @brief Default controller
struct NullController : public Controller {
public:
    /// @brief calculate motor outputs based on reference and estimate
    /// @param reference current target robot state
    /// @param estimate current estimate robot state
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs motor outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]) { }
};

/// @brief Position controller for the chassis
struct XDrivePositionController : public Controller {
private:
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
    XDrivePositionController() = default;

    /// @brief calculate motor outputs based on reference and estimate
    /// @param reference current target robot state
    /// @param estimate current estimate robot state
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs motor outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);

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
    XDriveVelocityController() = default;

    /// @brief take s in a micro_reference of wheel velocity
    /// @param reference current target robot state
    /// @param estimate current robot state estimate
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs current outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);

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
    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;

public:
    /// @brief default constructor
    YawController() = default;

    /// @brief calculate motor outputs based on reference and estimate
    /// @param reference current target robot state
    /// @param estimate current estimate robot state
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs motor outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);

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
    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;

public:
    PitchController() = default;
    /// @brief calculate motor outputs based on reference and estimate
    /// @param reference current target robot state
    /// @param estimate current estimate robot state
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs motor outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);

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
    /// @brief filter for controlling meters/second
    PIDFilter pid_high;
    /// @brief filter for controlling motor velocity
    PIDFilter pid_low;
public:
    /// @brief default constructor
    FlywheelController() = default;

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

/// @brief Controls the feeder
struct FeederController : public Controller {
private:
    /// @brief filter for controlling balls/sec
    PIDFilter pid_high;
    /// @brief filter for controlling motor velocity
    PIDFilter pid_low;

public:
    /// @brief default constructor
    FeederController() = default;

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

/// @brief Controller for the switcher, which is a fullstate controller with feedforward
struct SwitcherController : public Controller {
private:
    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;
public:
    /// @brief default
    SwitcherController() { }
    /// @brief calculate motor outputs based on reference and estimate
    /// @param reference current target robot state
    /// @param estimate current estimate robot state
    /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
    /// @param outputs motor outputs
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);

    /// @brief reset the controller
    inline void reset() {
        Controller::reset();
        pidp.sumError = 0.0;
        pidv.sumError = 0.0;
    }
};

#endif // CONTROLLER_H