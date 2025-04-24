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

/// @brief Controls the Engineer arm
struct EngineerArmController : public Controller {
    private:
        /// @brief PID for controlling pitch position
        PIDFilter pid_pitch_position;
        /// @brief PID for controlling pitch velocity
        PIDFilter pid_pitch_velocity;
        /// @brief PID for controlling Linear position
        PIDFilter pid_linear_position;
        /// @brief PID for controlling Linear velocity
        PIDFilter pid_linear_velocity;
        /// @brief PID for controlling pitch2 position
        PIDFilter pid_pitch2_position;
        /// @brief PID for controlling pitch2 velocity
        PIDFilter pid_pitch2_velocity;
        /// @brief PID for controlling yaw2 position
        PIDFilter pid_yaw2_position;
        /// @brief PID for controlling yaw2 velocity
        PIDFilter pid_yaw2_velocity;
        /// @brief PID for controlling pitch3 position
        PIDFilter pid_pitch3_position;
        /// @brief PID for controlling pitch3 velocity
        PIDFilter pid_pitch3_velocity;
        /// @brief PID for controlling roll position
        PIDFilter pid_roll_position;
        /// @brief PID for controlling roll velocity
        PIDFilter pid_roll_velocity;
        /// @brief PID for controlling yaw position
    
    public:
        /// @brief default constructor
        EngineerArmController() = default;
    
        /// @brief calculate motor outputs based on reference and estimate
        /// @param reference current target robot state
        /// @param estimate current estimate robot state
        /// @param micro_estimate current micro estimate robot state (state of motors, not joints)
        /// @param outputs motor outputs
        void step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float micro_estimate[CAN_MAX_MOTORS][MICRO_STATE_LEN], float outputs[CAN_MAX_MOTORS]);
    
        /// @brief reset the controller
        inline void reset() {
            Controller::reset();
            pid_pitch_position.sumError = 0.0;
            pid_pitch_velocity.sumError = 0.0;
        }

        /// @brief convert the state to the linkage input position
        /// @param state state to convert
        /// @return converted state
        /// @note see the matlab code at the bottom of the file for to generate the equation
        float to_pitch_linkage_angle(float state) {
            double a = (1087 * sin(state)) / 4;
            double b = (1087 * cos(state)) / 4;
            double c = pow(b + 140, 2);
            double d = pow(a + 72, 2);
            return atan(((75 * d + (a + 147) * (d + c - 150745.0 / 4) + (b + 140) * sqrt(162409 * d - pow(d + c + 150745.0 / 4, 2) + 162409 * c) + 75 * c + 11305875.0 / 4) / (2 * d + 2 * c) - 75)
             / ((180 * d + (b + 320) * (d + c - 150745.0 / 4) - (a + 72) * sqrt(162409 * d - pow(d + c + 150745.0 / 4, 2) + 162409 * c) + 180 * c + 6783525) / (2 * d + 2 * c) - 180));
        }

    /// @brief gets the effective gear ratio of the linkage at the given angle
    /// @param x input angle
    /// @return effective gear ratio
    /// @note this is the derivative of the to_pitch_linkage_angle function
    /// @note see the matlab code at the bottom of the file for to generate the equation
    float pitch_linkage_gear_ratio(float x) {
        const float s = sin(x);
        const float c = cos(x);

        const float A = s*2.7175e2+7.2e1;
        const float B = c*2.7175e2+1.4e2;
        
        const float A2 = (A*A);
        const float B2 = (B*B);
        
        const float C = A2+B2;

        const float cA = c*A;
        const float sB = s*B;

        const float D = (s*2.7175e2+1.47e2);
        const float E = (cA*5.435e2-sB*5.435e2);

        const float F = (C-3.768625e4);

        const float G = A2*1.62409e5;

        const float H = sqrt(G-pow(C+3.768625e4, 2)+B2*1.62409e5);

        const float I = A2*2.0+B2*2.0;

        const float J = cA*1.087e3-sB*1.087e3;
        const float K = A2*7.5e1+D*F+(B)*H+B2*7.5e1+2.82646875e6;
        const float L = A2*1.8e2+(c*2.7175e2+3.2e2)*F-(A)*H+B2*1.8e2+6.783525e6;

        const float result = (((E*D+c*F*2.7175e2+cA*4.07625e4-sB*4.07625e4-s*H*2.7175e2-((B)*(cA*(-8.82692915e7)+sB*8.82692915e7+E*(A2+B2+3.768625e4)*2.0)*1.0/sqrt(G-pow(C+3.768625e4,2)+B2*1.62409e5))/2.0)/(I)-1.0/pow(I,2)*(J)*(K))
            /((L)/(I)-1.8e2)-((E*(c*2.7175e2+3.2e2)-s*(A2+B2-3.768625e4)*2.7175e2-c*H*2.7175e2+cA*9.783e4-sB*9.783e4+((A)*(cA*(-8.82692915e7)+sB*8.82692915e7+E*(C+3.768625e4)*2.0)*1.0/H)/2.0)/(I)-1.0/pow(I, 2)*(J)*(L))
            *1.0/pow((L)/(I)-1.8e2, 2)*((K)/(I)-7.5e1))/(1.0/pow((L)/(I)-1.8e2, 2)*pow((K)/(I)-7.5e1, 2)+1.0);
    
        return result;
    }
    };

#endif // CONTROLLER_H
/*
Matlab code to generate the equation for to_pitch_linkage_angle and to_pitch_linkage_velocity
%base joint positions
x4 = 180;
y4 = 75;
x0 = 320;
y0 = 147;
%link lengths
l1 = 271.75;
l2 = 201.5;
l3 = 54;
a1 = l3^2 - l2^2;
C = @(a,b,c,d) (a-b).^2 + (c-d).^2;
m = -1;
syms x
eq = atan( ...
    ( y4 - ( ( (sin(x).*l1 + y0).*(a1+C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) - y4.*(a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) + m.*(x4 - (cos(x).*l1 + x0)).*sqrt(4.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0)).*l2.^2 - (a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))).^2) ) / (2.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) ) ) ...
    ./ ...
    ( x4 - ( ( (cos(x).*l1 + x0).*(a1+C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) - x4.*(a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) + m.*((sin(x).*l1 + y0) - y4).*sqrt(4.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0)).*l2.^2 - (a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))).^2) ) / (2.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) ) ) ...
);
simplify(eq)
diff(eq)
*/

