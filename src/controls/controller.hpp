#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"
#include "../comms/rm_can.hpp"
#include "../sensors/RefSystem.hpp"
#include "state.hpp"
#include "../sensors/ACS712.hpp"

#define NUM_GAINS 24
#define NUM_CONTROLLERS 12
#define NUM_CONTROLLER_LEVELS 3

/// @brief Parent controller struct, all controllers should be based off of this.
struct Controller {
protected:
    /// @brief gains for a specific controller
    float gains[NUM_GAINS];
    /// @brief ratio to help with between motors and joints
    float gear_ratios[NUM_MOTORS];
    /// @brief Timer object so we can use dt in controllers
    Timer timer;
    /// @brief defines controller inputs and outputs (0 means Macro_state input, micro_state output)
    /// @note (1 means Micro_state input, motor_current output) (2 means Macro state input, motor_current output)
    int controller_level;
public:
    /// @brief default constructor
    Controller() {};

    /// @brief set the gains for this specific controller
    /// @param _gains gains array of length NUM_GAINS
    void set_gains(const float _gains[NUM_GAINS]) {
        for (int i = 0; i < NUM_GAINS; i++)
            gains[i] = _gains[i];
    }

    /// @brief set the gear ratios for this specific controller
    /// @param _gear_ratios gains array of length NUM_GAINS
    void set_gear_ratios(float _gear_ratios[NUM_GAINS]) {
        for (int i = 0; i < NUM_GAINS; i++)
            gear_ratios[i] = _gear_ratios[i];
    }

    /// @brief Generates an output from a state reference and estimation
    /// @param reference reference (target state)
    /// @param estimate current macro state estimate
    /// @param micro_estimate current micro state estimate
    /// @param outputs array of new motor inputs
    virtual void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]);

    /// @brief Resets integrators/timers
    virtual void reset() { timer.start_timer(); }
};

/// @brief Default controller
struct NullController : public Controller {
public:
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]) {}
};

/// @brief Controller for all chassis movement, which includes power limiting
struct XDriveVelocityController : public Controller {
private:
    /// @brief filter for calculating pid controller outputs
    PIDFilter pid;

public:
    /// @brief set controller level and make sure it's a low level controller
    /// @param _controller_level controller level(if it outputs a torque or a target micro state).
    XDriveVelocityController() = default;

    /// @brief take s in a micro_reference of wheel velocity
    /// @param reference reference
    /// @param estimate estimate
    /// @return outputs motor current
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]);

    void reset() {
        Controller::reset();
        pid.sumError = 0.0;
    }
};

/// @brief Position controller for the chassis
struct XDriveChassisPositionController : public Controller {
private:
    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp[3];
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv[3];
    /// @brief outputs of the pid position controller
    float outputp[i];
    /// @brief outputs of the pid velocity controller
    float outputv[i];
    /// @brief combined outputs of the pid position and velocity controllers
    float output[i];
    /// @brief target motor velocity
    float motor_velocity[4];
public:
    /// @brief set controller level and make sure it's not low level
    /// @param _controller_level controller level(if it outputs a torque or a target micro state).
    XDrivePositionController() = defualt;

    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]);

    void reset() {
        Controller::reset();
        pidp.sumError = 0.0;
    }
};

struct YawController : public Controller {
    private:
        PIDFilter pidp;
        PIDFilter pidv;

    public:
        YawController() = default;

        void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]);

        void  reset(){
            Controller::reset();
            pidp.sumError = 0.0;
            pidv.sumError = 0.0;
        }
}

struct PitchController : public Controller {

    private:
        PIDFilter pidp;
        PIDFilter pidv;

    public:
        PitchController() = default;

        void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]);

        void  reset(){
            Controller::reset();
            pidp.sumError = 0.0;
            pidv.sumError = 0.0;
        }
}

struct FlywheelController : public Controller{
    private:
        PIDFilter pidv;

    public:
        FlywheelController() = default;

        void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]);

        void reset(){
            Controller:reset();
            pidv.sumError = 0.0;
        }
}

struct FeederController : public Controller{
    private:
        PIDFilter pidv;

    public:
        FeederController() = default;

        void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]);

        void reset(){
            Controller:reset();
            pidv.sumError = 0.0;
        }
}

/// @brief Controller for the switcher, which is a fullstate controller with feedforward
struct SwitcherController : public Controller {
private:
    /// @brief filter for calculating pid position controller outputs
    PIDFilter pidp;
    /// @brief filter for calculating pid velocity controller outputs
    PIDFilter pidv;
public:
    /// @brief set controller level and make sure it's not low level
    SwitcherController() {
    }
    /// @brief don't do anything if we get a macro state
    /// @param reference reference
    /// @param estimate estimate
    /// @return 0
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[NUM_MOTORS]);

    void reset() {
        Controller::reset();
        pidp.sumError = 0.0;
        pidv.sumError = 0.0;
    }
};

#endif // CONTROLLER_H