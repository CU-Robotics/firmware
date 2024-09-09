#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"
#include "../comms/rm_can.hpp"
#include "../sensors/RefSystem.hpp"
#include "state.hpp"
#include "../sensors/ACS712.hpp"

#define NUM_GAINS 12
#define NUM_CONTROLLERS 12
#define NUM_CONTROLLER_LEVELS 3

/// @brief Parent controller struct, all controllers should be based off of this.
struct Controller {
protected:
    /// @brief gains for a specific controller
    float gains[NUM_GAINS];
    /// @brief Timer object so we can use dt in controllers
    Timer timer;
    /// @brief defines controller inputs and outputs (0 means Macro_state input, micro_state output)
    /// @note (1 means Micro_state input, motor_current output) (2 means Macro state input, motor_current output)
    int controller_level;

    /// @brief ratio to help with between motors and joints
    float gear_ratio = 0;
public:
    /// @brief default constructor
    Controller() {};

    /// @brief set the gains for this specific controller
    /// @param _gains gains array of length NUM_GAINS
    void set_gains(float _gains[NUM_GAINS]) {
        for (int i = 0; i < NUM_GAINS; i++)
            gains[i] = _gains[i];
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
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[STATE_LEN][3]) {}
};

/// @brief Fullstate controller which works on both position and velocity of a state.
struct FullStateFeedbackController : public Controller {
private:
    /// @brief pid filter for calculating controller outputs
    PIDFilter pid1;
    /// @brief pid filter for calculating controller outputs
    PIDFilter pid2;

public:
    /// @brief this controller can work on position and velocity at the same time
    /// @param _controller_level controller level, which cannot be a low level controller
    FullStateFeedbackController(int _controller_level) {
        controller_level = _controller_level;
        if (controller_level == 1)
            Serial.println("FullStateFeedbackController can't be a low level controller");
    }

    float step(float reference[3], float estimate[3]) {
        float dt = timer.delta();
        float output = 0.0;

        pid1.K[0] = gains[0];
        pid1.K[1] = gains[1];
        pid1.K[2] = gains[2];
        pid1.K[3] = gains[3] * sin(reference[0]);
        pid2.K[0] = gains[4];
        pid2.K[1] = gains[5];
        pid2.K[2] = gains[6];

        pid1.setpoint = reference[0];
        pid1.measurement = estimate[0];

        pid2.setpoint = reference[1];
        pid2.measurement = estimate[1];

        output += pid1.filter(dt, true, true); // position wraps
        output += pid2.filter(dt, true, false); // no wrap for velocity
        output = constrain(output, -1.0, 1.0);
        return output;
    }

    float step(float reference, float estimate[MICRO_STATE_LEN]) { return 0; }

    void reset() {
        Controller::reset();
        pid1.sumError = 0.0;
        pid2.sumError = 0.0;
    }
};

/// @brief Controller for all chassis movement, which includes power limiting
struct XDriveVelocityController : public Controller {
private:
    /// @brief filter for calculating pid controller outputs
    PIDFilter pid;

public:
    /// @brief set controller level and make sure it's a low level controller
    /// @param _controller_level controller level(if it outputs a torque or a target micro state).
    ChassisPIDVelocityController(int _controller_level) {
        controller_level = _controller_level;
        if (controller_level != 1)
            Serial.println("chassisPIDVelocityController must be a low level controller");
    }

    /// @brief take s in a micro_reference of wheel velocity
    /// @param reference reference
    /// @param estimate estimate
    /// @return outputs motor current
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[STATE_LEN][3]);

    void reset() {
        Controller::reset();
        pid.sumError = 0.0;
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
    /// @brief set controller level and make sure it's not low level
    SwitcherController() {
    }
    /// @brief don't do anything if we get a macro state
    /// @param reference reference
    /// @param estimate estimate
    /// @return 0
    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[STATE_LEN][3]);
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
    ChassisFullStateFeedbackController() {
        controller_level = _controller_level;
        if (controller_level == 1)
            Serial.println("ChassisFullStateFeedbackController must not be a low level controller");
    }

    void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[STATE_LEN][3]);

    void reset() {
        Controller::reset();
        pidp.sumError = 0.0;
    }
};


struct YawController : public Controller {

    private:

    public:
        YawController() = default;

        void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[NUM_MOTORS][MICRO_STATE_LEN], float outputs[STATE_LEN][3]);

        void
}

#endif // CONTROLLER_H