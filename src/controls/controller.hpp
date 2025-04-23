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
            double a = (1087 * sin(x)) / 4;
            double b = (1087 * cos(x)) / 4;
            double c = pow(b + 140, 2);
            double d = pow(a + 72, 2);
            return atan(((75 * d + (a + 147) * (d + c - 150745.0 / 4) + (b + 140) * sqrt(162409 * d - pow(d + c + 150745.0 / 4, 2) + 162409 * c) + 75 * c + 11305875.0 / 4) / (2 * d + 2 * c) - 75)
             / ((180 * d + (b + 320) * (d + c - 150745.0 / 4) - (a + 72) * sqrt(162409 * d - pow(d + c + 150745.0 / 4, 2) + 162409 * c) + 180 * c + 6783525) / (2 * d + 2 * c) - 180));
        }

    /// @brief convert the state to the linkage input velocity
    /// @param state state to convert
    /// @return converted state
    /// @note see the matlab code at the bottom of the file for to generate the equation
    float to_pitch_linkage_velocity(float state) {
       // Precompute sin(x) and cos(x)
        double sin_x = std::sin(x);
        double cos_x = std::cos(x);

        // Precompute frequently used terms
        double A = (1087.0 * sin_x) / 4.0 + 72.0;
        double B = (1087.0 * cos_x) / 4.0 + 140.0;
        double A2 = std::pow(A, 2);
        double B2 = std::pow(B, 2);

        // Common sum of squares
        double sum_squares = A2 + B2;

        // Precompute the constant adjustment
        double sum_squares_minus = sum_squares + 150745.0 / 4.0;

        // Precompute the massive radical term
        double radical = std::sqrt(162409.0 * A2 - std::pow(sum_squares_minus, 2) + 162409.0 * B2);

        // Precompute more common terms
        double denom = 2.0 * (A2 + B2);

        // Precompute outer denominators
        double denom_full = denom - 180.0;

        // Precompute big numerator piece
        double big_numerator_piece = (
            180.0 * A2 +
            (B + 180.0) * (sum_squares - 150745.0 / 4.0) -
            A * radical +
            180.0 * B2 +
            6783525.0
        ) / denom - 180.0;

        // Precompute small term
        double small_term = (
            75.0 * A2 +
            A * (sum_squares - 150745.0 / 4.0) +
            B * radical +
            75.0 * B2 +
            11305875.0 / 4.0
        ) / denom - 75.0;

        // Precompute another large factor
        double big_factor = (
            (176538583.0 * sin_x * B) / 2.0 -
            (176538583.0 * cos_x * A) / 2.0 +
            2.0 * ((1087.0 * cos_x * A) / 2.0 - (1087.0 * sin_x * B) / 2.0) * sum_squares_minus
        );

        // Now build up the top expression
        double top_expr = (
            (1087.0 * cos_x * A - 1087.0 * sin_x * B) / 2.0 * (A + 75.0)
            + (1087.0 * cos_x * sum_squares_minus) / 4.0
            + (81525.0 * cos_x * A) / 2.0
            - (81525.0 * sin_x * B) / 2.0
            - (1087.0 * sin_x * radical) / 4.0
            - ((B * big_factor) / (2.0 * radical)) / denom
            - ((1087.0 * cos_x * A - 1087.0 * sin_x * B) * (small_term)) / (std::pow(denom, 2))
        );

        // Build bottom expression
        double bottom_expr = (
            big_numerator_piece
            - (top_expr * small_term) / (std::pow(denom_full, 2))
        ) / (
            (std::pow(small_term, 2) / std::pow(denom_full, 2)) + 1.0
        );

        // Finally divide top by bottom
        double result = top_expr / bottom_expr;

        return result;
    }
    };

#endif // CONTROLLER_H
/*
Matlab code to generate the equation for to_pitch_linkage_angle and to_pitch_linkage_velocity
x4 = 180;
y4 = 75;
x0 = 320;
y0 = 147;
l1 = 271.75;
l2 = 201.5;
l3 = 54;
a1 = l3^2 - l2^2;
C = @(a,b,c,d) (a-b).^2 + (c-d).^2;
m = -1;
syms x
eq = atan( ...
    ( y4 - ( ( (sin(x).*l1 + y0).*(a1+C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) - y4.*(a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) + m.*(x4 - (cos(x).*l1 + x0)).*sqrt(4.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0)).*l2.^2 - (a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))).^2) ) ./ (2.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) ) ) ...
    ./ ...
    ( x4 - ( ( (cos(x).*l1 + x0).*(a1+C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) - x4.*(a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) + m.*((sin(x).*l1 + y0) - y4).*sqrt(4.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0)).*l2.^2 - (a1-C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))).^2) ) ./ (2.*C(x4,(cos(x).*l1 + x0),y4,(sin(x).*l1 + y0))) ) ) ...
);
simplify(eq)
diff(eq)
*/

