#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"
#include "../comms/rm_can.hpp"
#include "../sensors/RefSystem.hpp"
#include "state.hpp"
#include "../sensors/ACS712.hpp"

#define NUM_GAINS 12
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
    /// @param reference target reference
    /// @param estimate current macro state estimate
    /// @returns motor_current or Micro_state depending on controller_level
    virtual float step(float reference[3], float estimate[3]);
    /// @brief Generates an output from a state reference and estimation
    /// @param reference target reference
    /// @param estimate current micro state estimate
    /// @returns motor_current 
    virtual float step(float reference, float estimate[MICRO_STATE_LEN]);

    /// @brief Resets integrators/timers
    virtual void reset() { timer.start_timer(); }
};

/// @brief Default controller
struct NullController : public Controller {
public:
    float step(float reference[3], float estimate[3]) { return 0; }

    float step(float reference, float estimate[MICRO_STATE_LEN]) { return 0; }
};

/// @brief PID controller working on position
struct PIDPositionController : public Controller {
private:
    /// @brief filter for calculating pid ouput
    PIDFilter pid;

public:
    /// @brief new position controller
    /// @param _controller_level controller level, whether it outputs target motor torque or target micro state
    PIDPositionController(int _controller_level) {
        controller_level = _controller_level;
    }
    /// @brief step for macro_state input
    /// @param reference macro_reference
    /// @param estimate macro_estimate
    /// @return Motor output or micro_reference
    float step(float reference[3], float estimate[3]) {
        float dt = timer.delta();

        pid.setpoint = reference[0]; // 0th index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];
        bool bounded = (controller_level == 2);
        float output = pid.filter(dt, bounded, false);
        return output;
    }
    /// @brief step for micro_state input
    /// @param reference micro_reference
    /// @param estimate micro_estimate
    /// @return Motor output
    float step(float reference, float estimate[MICRO_STATE_LEN]) {
        float dt = timer.delta();

        pid.setpoint = reference; // 0th index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];

        float output = pid.filter(dt, true, false);
        return output;
    }

    /// @brief reset controller which 0's integrators
    void reset() {
        Controller::reset();
        pid.sumError = 0.0;
    }
};

/// @brief PID controller working on velocity
struct PIDVelocityController : public Controller {
private:
    /// @brief filter for calculating pid ouput
    PIDFilter pid;

public:

    /// @brief Set controller level
    /// @param _controller_level Controller level, whether it outputs a target motor torque or a micro/macro state
    PIDVelocityController(int _controller_level) {
        controller_level = _controller_level;
    }
    /// @brief step for macro_state input
    /// @param reference macro_reference
    /// @param estimate macro_estimate
    /// @return Motor output or micro_reference
    float step(float reference[3], float estimate[3]) {
        float dt = timer.delta();

        pid.setpoint = reference[1]; // 1st index = position
        pid.measurement = estimate[1];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];

        bool bounded = (controller_level == 2);
        float output = pid.filter(dt, bounded, false);
        return output;
    }
    /// @brief step for micro_state input
    /// @param reference micro_reference
    /// @param estimate micro_estimate
    /// @return Motor output
    float step(float reference, float estimate[MICRO_STATE_LEN]) {
        float dt = timer.delta();

        pid.setpoint = reference; // 0th index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];

        float output = pid.filter(dt, true, false);
        return output;
    }

    /// @brief reset controller which 0's integrators
    void reset() {
        Controller::reset();
        pid.sumError = 0.0;
    }
};

/// @brief PID controller with feedforward capability
struct PIDFVelocityController : public Controller {
private:
    /// @brief pid filter for calculating controller outputs
    PIDFilter pid;

public:
    /// @brief new controller
    /// @param _controller_level controller level, whether it ouputs target motor torque or micro state
    PIDFVelocityController(int _controller_level) {
        controller_level = _controller_level;
    }
    /// @brief step for macro_state input
    /// @param reference macro_reference
    /// @param estimate macro_estimate
    /// @return Motor output or micro_reference
    float step(float reference[3], float estimate[3]) {
        float dt = timer.delta();

        pid.setpoint = reference[1]; // 1st index = position
        pid.measurement = estimate[1];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];
        pid.K[3] = reference[1];

        bool bounded = (controller_level == 2);
        float output = pid.filter(dt, bounded, false);
        return output;
    }
    /// @brief step for micro_state input
    /// @param reference micro_reference
    /// @param estimate micro_estimate
    /// @return Motor output
    float step(float reference, float estimate[MICRO_STATE_LEN]) {
        float dt = timer.delta();

        pid.setpoint = reference; // 0th index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];

        float output = pid.filter(dt, true, false);
        return output;
    }

    void reset() {
        Controller::reset();
        pid.sumError = 0.0;
    }
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
struct ChassisPIDVelocityController : public Controller {
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
    /// @brief don't do anything if we get a macro state
    /// @param reference reference
    /// @param estimate estimate
    /// @return 0
    float step(float reference[3], float estimate[3]) { return 0; }
    /// @brief take s in a micro_reference of wheel velocity
    /// @param reference reference
    /// @param estimate estimate
    /// @return outputs motor current
    float step(float reference, float estimate[MICRO_STATE_LEN]) {
        float dt = timer.delta();
        pid.setpoint = reference; // 1st index = position
        pid.measurement = estimate[0];
        pid.K[0] = gains[0];
        pid.K[1] = gains[1];
        pid.K[2] = gains[2];
        float power_buffer = ref.ref_data.robot_power_heat.buffer_energy;
        
        // Power limiting
        
        float power_limit_ratio = 1.0;
        float power_buffer_limit_thresh = gains[3];
        float power_buffer_critical_thresh = gains[4];
        if (power_buffer < power_buffer_limit_thresh) {
            power_limit_ratio = constrain(((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh), 0.0, 1.0);
        }
        float output = pid.filter(dt, true, false) * power_limit_ratio;
        return output;
    }

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
    /// @param _controller_level controller level(if it outputs a torque or a target micro state).
    SwitcherController(int _controller_level) {
        controller_level = _controller_level;
        if (controller_level == 1)
            Serial.println("SwitcherController must not be a low level controller");
    }
    /// @brief don't do anything if we get a macro state
    /// @param reference reference
    /// @param estimate estimate
    /// @return 0
    float step(float reference[3], float estimate[3]) { 
        float dt = timer.delta();
        pidp.setpoint = reference[0]; // 1st index = position
        pidp.measurement = estimate[0];

        pidp.K[0] = gains[0];
        pidp.K[1] = gains[1];
        pidp.K[2] = gains[2];
        // Feed forward to push the switcher into the wall constantly with a small force
        if(reference[0] > .95) pidp.K[3] = gains[3];
        else if(reference[0] < -.95) pidp.K[3] = -gains[3];
        else pidp.K[3] = 0;

        pidv.setpoint = reference[1]; 
        pidv.measurement = estimate[1];

        pidv.K[0] = gains[4];
        pidv.K[1] = gains[5];
        pidv.K[2] = gains[6];
        float outputp = pidp.filter(dt, true, false);
        float outputv = pidv.filter(dt, true, false);
        float output = outputp + outputv;
        return output;
    }


    /// @brief dont do anything if we get a micro_reference
    /// @param reference reference
    /// @param estimate estimate
    /// @return 0
    float step(float reference, float estimate[MICRO_STATE_LEN]) {return 0;}

    void reset() {
        Controller::reset();
        pidp.sumError = 0.0;
    }
};

#endif // CONTROLLER_H