#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include "sensors/can/can_manager.hpp"
#include "state.hpp"
#include "sensors/RefSystem.hpp"
#include "comms/config_layer.hpp"

#include "sensors/buff_encoder.hpp"
#include "sensors/rev_encoder.hpp"
#include "sensors/ICM20649.hpp"
#include "sensors/TOFSensor.hpp"
#include "sensors/SensorManager.hpp"


/// @brief Parent estimator struct. All estimators should inherit from this.
struct Estimator {
public:
    Estimator() { };

    virtual ~Estimator() { };

    /// @brief step the current state(s) and update the estimate array accordingly
    /// @param outputs estimated state array to update with certain estimated states
    /// @param curr_state current state array to update with new state
    /// @param override true if we want to override the current state with the new state
    virtual void step_states(float outputs[STATE_LEN][3], float curr_state[STATE_LEN][3], int override) = 0;

    /// @brief bool that indicates if the estimator is a micro or macro estimator
    bool micro_estimator = false;

protected:
    ///@brief create a timer object for each estimator
    Timer time;

    /// @brief Computes the magnitude of a vector given length n
    /// @param a Vector to compute the magnitude of
    /// @param n Length of Vector a
    /// @return returns the magnitude of a
    float __magnitude(float* a, int n) {
        float square_sum = 0;
        for (int i = 0; i < n; i++) {
            square_sum += pow(a[i], 2);
        }
        square_sum = sqrt(square_sum);
        return square_sum;
    }

    /// @brief Computes the dot product of 2 vectors with a given length (nx1)
    /// @param a Vector A
    /// @param b Vector B
    /// @param n Length of A and B (must be the same length)
    /// @return returns Dot product solution (scalar)
    float __vectorProduct(float* a, float* b, int n) {
        float product = 0;
        for (int i = 0; i < n; i++) {
            product += a[i] * b[i];
        }
        return product;
    }

    /// @brief Computes the cross product of 2 given vectors of length 3
    /// @param v_A Vector A (3x1)
    /// @param v_B Vector B (3x1)
    /// @param output Cross product output vector (3x1)
    void __crossProduct(float v_A[ ], float v_B[ ], float output[ ]) {
        output[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
        output[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
        output[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
    }

    /// @brief Rotates input_vector around the given unit_vector by theta radians
    /// @param unit_vector Vector to rotate around
    /// @param input_vector Vector to be rotated
    /// @param theta Angle to rotate (Rad)
    /// @param output New rotated vector
    void __rotateVector3D(float unit_vector[ ], float input_vector[ ], float theta, float output[ ]) {
        float unit_cross_input[3];
        __crossProduct(unit_vector, input_vector, unit_cross_input);
        output[0] = (input_vector[0] * cos(theta)) + (unit_cross_input[0] * sin(theta)) + (unit_vector[0] * __vectorProduct(unit_vector, input_vector, 3) * (1 - cos(theta)));
        output[1] = (input_vector[1] * cos(theta)) + (unit_cross_input[1] * sin(theta)) + (unit_vector[1] * __vectorProduct(unit_vector, input_vector, 3) * (1 - cos(theta)));
        output[2] = (input_vector[2] * cos(theta)) + (unit_cross_input[2] * sin(theta)) + (unit_vector[2] * __vectorProduct(unit_vector, input_vector, 3) * (1 - cos(theta)));
    }

    /// @brief This functions finds the determinant of a 3x3 Matrix
    ///@param mat matrix to find determinant of
    ///@return calculated determinant
    float determinantOfMatrix(float mat[3][3]) {
        float ans;
        ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
        return ans;
    }

    /// @brief This function finds the solution of a 3x3 system of linear equations using cramer's rule.
    /// @param coeff 3x3 coeff matrix for the system with 3x1 solution matrix added to the end
    /// @param output 3x1 Array for the solutions
    void solveSystem(float coeff[3][4], float output[3]) {
        // Matrix d using coeff as given in cramer's rule
        float d[3][3] = {
            {coeff[0][0], coeff[0][1], coeff[0][2]},
            {coeff[1][0], coeff[1][1], coeff[1][2]},
            {coeff[2][0], coeff[2][1], coeff[2][2]}
        };
        // Matrix d1 using coeff as given in cramer's rule
        float d1[3][3] = {
            {coeff[0][3], coeff[0][1], coeff[0][2]},
            {coeff[1][3], coeff[1][1], coeff[1][2]},
            {coeff[2][3], coeff[2][1], coeff[2][2]},
        };
        // Matrix d2 using coeff as given in cramer's rule
        float d2[3][3] = {
            {coeff[0][0], coeff[0][3], coeff[0][2]},
            {coeff[1][0], coeff[1][3], coeff[1][2]},
            {coeff[2][0], coeff[2][3], coeff[2][2]},
        };
        // Matrix d3 using coeff as given in cramer's rule
        float d3[3][3] = {
            {coeff[0][0], coeff[0][1], coeff[0][3]},
            {coeff[1][0], coeff[1][1], coeff[1][3]},
            {coeff[2][0], coeff[2][1], coeff[2][3]},
        };

        // Calculating Determinant of Matrices d, d1, d2, d3
        float D = determinantOfMatrix(d);
        float D1 = determinantOfMatrix(d1);
        float D2 = determinantOfMatrix(d2);
        float D3 = determinantOfMatrix(d3);

        // Case 1
        if (D != 0) {
            // Coeff have a unique solution
            output[0] = D1 / D;
            output[1] = D2 / D;
            output[2] = D3 / D;
        }
        // Case 2
        else {
            if (D1 == 0 && D2 == 0 && D3 == 0) {
                output[0] = 0;
                output[1] = 0;
                output[2] = 0;
            } else if (D1 != 0 || D2 != 0 || D3 != 0) {
                output[0] = 0;
                output[1] = 0;
                output[2] = 0;
            }
        }
    }
};

/// @brief Estimate the yaw, pitch, and chassis heading
struct GimbalEstimator : public Estimator {
private:
    /// @brief yaw encoder offset for 0 radians
    float YAW_ENCODER_OFFSET; // input variables
    /// @brief yaw encoder offset for 0 radians
    float PITCH_ENCODER_OFFSET;
    /// @brief calculated pitch angle
    float pitch_angle;
    /// @brief calculated yaw angle
    float yaw_angle;
    /// @brief calculated roll angle
    float roll_angle;
    /// @brief calculated chassis angle
    float chassis_angle;

    /// @brief yaw imu vector
    float imu_yaw_axis_vector[3];

    /// @brief pitch imu vector
    float imu_pitch_axis_vector[3];

    /// @brief gravity pitch angle
    float starting_pitch_angle;
    /// @brief yaw axis in spherical coords
    float yaw_axis_spherical[3];

    /// @brief pitch axis in spherical coords
    float pitch_axis_spherical[3];

    /// @brief roll axis in spherical coords
    float roll_axis_spherical[3];

    /// @brief yaw axis unit vector
    float yaw_axis_unitvector[3];

    /// @brief pitch axis unit vector
    float pitch_axis_unitvector[3];

    /// @brief roll axis unit vector
    float roll_axis_unitvector[3];

    /// @brief global relative yaw
    float yaw_axis_global[3];

    /// @brief global relative pitch
    float pitch_axis_global[3];

    /// @brief global relative roll
    float roll_axis_global[3];
    /// @brief current calculated yaw velocity
    float current_yaw_velocity = 0;
    /// @brief previous calculated yaw velocity
    float previous_yaw_velocity = 0;
    /// @brief current calculated pitch velocity
    float current_pitch_velocity = 0;
    /// @brief previous calculated pitch velocity
    float previous_pitch_velocity = 0;
    /// @brief current calculated roll velocity
    float current_roll_velocity = 0;
    /// @brief previous calculated roll velocity
    float previous_roll_velocity = 0;
    /// @brief global relative yaw velocity
    float global_yaw_velocity = 0;
    /// @brief global relative roll velocity
    float global_roll_velocity = 0;
    /// @brief global relative pitch velocity
    float global_pitch_velocity = 0;
    /// @brief global pitch angle
    float global_pitch_angle = 1.92;
    /// @brief global yaw angle
    float global_yaw_angle = 0;
    /// @brief global roll angle
    float global_roll_angle = 0;
    /// @brief current rev encoder raw value
    float curr_rev_raw[3] = { 0 };

    /// @brief previous rev encoder raw value
    float prev_rev_raw[3] = { 0 };

    /// @brief total meters travelled by each odom wheel
    float total_odom_pos[3] = { 0 };

    /// @brief rev encoder difference
    float rev_diff[3] = { 0 };

    /// @brief odom pos difference
    float odom_pos_diff[3] = { 0 };
    /// @brief previous chassis angle
    float prev_chassis_angle = 0;
    /// @brief odom pod offset from the center of the robot
    float odom_axis_offset_x;
    /// @brief odom pod offset from the center of the robot
    float odom_axis_offset_y;
    /// @brief odom pod angle offset radians
    float odom_angle_offset = 0.1745; // 10 degrees
    // float odom_angle_offset = 0;
    /// @brief odom wheel radius
    float odom_wheel_radius;
    /// @brief initial chassis angle
    float initial_chassis_angle = 0;
    /// @brief counts one time to set the starting chassis angle
    int count1 = 0;
    /// @brief delta time
    float dt = 0;

    /// @brief buff encoder on the yaw
    BuffEncoder* buff_enc_yaw;
    /// @brief buff encoder on the pitch
    BuffEncoder* buff_enc_pitch;
    /// @brief Odom encoder values
    RevEncoder* rev_enc[3];
    /// @brief can pointer from EstimatorManager
    CANManager* can;
    /// @brief icm imu
    ICM20649* icm_imu;

    /// @brief position estimate to store position after integrating used for chassis odometry
    float pos_estimate[3] = { 0,0,0 };

    /// @brief previous pose to store the previous pose for chassis odometry
    float previous_pos[3] = { 0,0,0 };

public:
    /// @brief estimate the state of the gimbal
    /// @param config_data inputted sensor values from khadas yaml
    /// @param sensor_manager sensor manager object 
    /// @param can can data from Estimator Manager
    GimbalEstimator(Config config_data, SensorManager* sensor_manager, CANManager* can);
  
    /// @brief calculate estimated states and add to output array
    /// @param output output array to add estimated states to
    /// @param curr_state current state array to update with new state
    /// @param override true if we want to override the current state with the new state
    void step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override) override;
};

/// @brief Estimate the yaw, pitch, and chassis heading
struct GimbalEstimatorNoOdom : public Estimator {
private:
    /// @brief yaw encoder offset for 0 radians
    float YAW_ENCODER_OFFSET; // input variables
    /// @brief yaw encoder offset for 0 radians
    float PITCH_ENCODER_OFFSET;
    /// @brief calculated pitch angle
    float pitch_angle;
    /// @brief calculated yaw angle
    float yaw_angle;
    /// @brief calculated roll angle
    float roll_angle;
    /// @brief calculated chassis angle
    float chassis_angle;

    /// @brief yaw imu vector
    float imu_yaw_axis_vector[3];

    /// @brief pitch imu vector
    float imu_pitch_axis_vector[3];

    /// @brief gravity pitch angle
    float starting_pitch_angle;
    /// @brief yaw axis in spherical coords
    float yaw_axis_spherical[3];

    /// @brief pitch axis in spherical coords
    float pitch_axis_spherical[3];

    /// @brief roll axis in spherical coords
    float roll_axis_spherical[3];

    /// @brief yaw axis unit vector
    float yaw_axis_unitvector[3];

    /// @brief pitch axis unit vector
    float pitch_axis_unitvector[3];

    /// @brief roll axis unit vector
    float roll_axis_unitvector[3];

    /// @brief global relative yaw
    float yaw_axis_global[3];

    /// @brief global relative pitch
    float pitch_axis_global[3];

    /// @brief global relative roll
    float roll_axis_global[3];
    /// @brief current calculated yaw velocity
    float current_yaw_velocity = 0;
    /// @brief previous calculated yaw velocity
    float previous_yaw_velocity = 0;
    /// @brief current calculated pitch velocity
    float current_pitch_velocity = 0;
    /// @brief previous calculated pitch velocity
    float previous_pitch_velocity = 0;
    /// @brief current calculated roll velocity
    float current_roll_velocity = 0;
    /// @brief previous calculated roll velocity
    float previous_roll_velocity = 0;
    /// @brief global relative yaw velocity
    float global_yaw_velocity = 0;
    /// @brief global relative roll velocity
    float global_roll_velocity = 0;
    /// @brief global relative pitch velocity
    float global_pitch_velocity = 0;
    /// @brief global pitch angle
    float global_pitch_angle = 1.92;
    /// @brief global yaw angle
    float global_yaw_angle = 0;
    /// @brief global roll angle
    float global_roll_angle = 0;
    /// @brief current rev encoder raw value
    float curr_rev_raw[3] = { 0 };

    /// @brief previous rev encoder raw value
    float prev_rev_raw[3] = { 0 };

    /// @brief total meters travelled by each odom wheel
    float total_odom_pos[3] = { 0 };

    /// @brief rev encoder difference
    float rev_diff[3] = { 0 };

    /// @brief odom pos difference
    float odom_pos_diff[3] = { 0 };
    /// @brief previous chassis angle
    float prev_chassis_angle = 0;
    /// @brief odom pod offset from the center of the robot
    float odom_axis_offset_x;
    /// @brief odom pod offset from the center of the robot
    float odom_axis_offset_y;
    /// @brief odom pod angle offset radians
    float odom_angle_offset = 0.1745; // 10 degrees
    // float odom_angle_offset = 0;
    /// @brief odom wheel radius
    float odom_wheel_radius;
    /// @brief initial chassis angle
    float initial_chassis_angle = 0;
    /// @brief counts one time to set the starting chassis angle
    int count1 = 0;
    /// @brief delta time
    float dt = 0;

    /// @brief buff encoder on the yaw
    BuffEncoder* buff_enc_yaw;
    /// @brief buff encoder on the pitch
    BuffEncoder* buff_enc_pitch;

    /// @brief can pointer from EstimatorManager
    CANManager* can;
    /// @brief icm imu
    ICM20649* icm_imu;

    /// @brief position estimate to store position after integrating used for chassis odometry
    float pos_estimate[3] = { 0,0,0 };

public:
    /// @brief estimate the state of the gimbal
    /// @param config_data inputted sensor values from khadas yaml
    /// @param sensor_manager sensor manager object 
    /// @param can can from Estimator Manager
    GimbalEstimatorNoOdom(Config config_data, SensorManager* sensor_manager, CANManager* can);

    GimbalEstimatorNoOdom() { };

    /// @brief calculate estimated states and add to output array
    /// @param output output array to add estimated states to
    /// @param curr_state current state of the system
    /// @param override override the current state
    void step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override) override;
};

/// @brief Estimate the state of the flywheels as meters/second of balls exiting the barrel.
struct FlyWheelEstimator : public Estimator {
private:
    /// @brief can pointer from EstimatorManager
    CANManager* can;
    /// @brief calculated flywheel state from ref
    float projectile_speed_ref;

    /// @brief calculated flywheel state from can
    float linear_velocity;
    /// @brief can weight for weighted average
    float can_weight = 1;

    /// @brief ref weight for weighted average
    float ref_weight = 0;

public:
    /// @brief make new flywheel estimator and set can data pointer and num states
    /// @param can can pointer from EstimatorManager
    FlyWheelEstimator(CANManager* can);

    ~FlyWheelEstimator() { };

    /// @brief generate estimated states and replace in output array
    /// @param output array to be updated with the calculated states
    /// @param curr_state current state of the flywheel
    /// @param override override flag
    void step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override);
};

/// @brief Estimate the state of the feeder in balls/s
struct FeederEstimator : public Estimator {
private:
    /// @brief can pointer from EstimatorManager
    CANManager* can;

    /// @brief balls per second calculated from ref
    float balls_per_second_ref;

    /// @brief balls per second calculated from can
    float balls_per_second_can;

    /// @brief can weight for weighted average
    float can_weight = 1;

    /// @brief ref weight for weighted average
    float ref_weight = 0;

public:
    /// @brief make new feeder estimator and set can_data pointer and num_states
    /// @param can can pointer from EstimatorManager
    FeederEstimator(CANManager* can);

    ~FeederEstimator() { };

    /// @brief calculate state updates
    /// @param output updated balls per second of feeder
    /// @param curr_state current state of the feeder
    /// @param override override flag
    void step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override) override;
};

/// @brief Estimate the state of the switcher in millimeters
struct SwitcherEstimator : public Estimator {
private:
    /// @brief can pointer from EstimatorManager
    CANManager* can;

    /// @brief TOF sensor pointer from EstimatorManager
    TOFSensor* time_of_flight;

    /// @brief time of flight sensor offset
    float tof_sensor_offset = 0;

    /// @brief used to scale the tof sensor data to -1 to 1
    float tof_scale = 0;

    /// @brief last motor angle
    float last_motor_angle = 0;

    /// @brief total motor angle
    float total_motor_angle = 0;

    /// @brief delta time
    float dt = 0;

    /// @brief count to check if dt is valid
    int count = 0;
public:
    /// @brief make new barrel switcher estimator and set can_data pointer and num_states
    /// @param config config data from yaml
    /// @param can can pointer from EstimatorManager
    /// @param tof time of flight sensor object
    SwitcherEstimator(Config config, CANManager* can, TOFSensor* tof);

    /// @brief calculate state updates
    /// @param output updated balls per second of feeder
    /// @param curr_state current state of the barrel switcher
    /// @param override override flag
    void step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override);
};

/// @brief This estimator estimates our "micro" state which is stores all the motor velocities(in rad/s), whereas the other estimators estimate "macro" state which stores robot joints
struct LocalEstimator : public Estimator {
private:
    /// @brief can from EstimatorManager
    CANManager* can;


public:
    /// @brief Make new local estimator and set can data pointer and num states
    /// @param can can pointer from EstimatorManager
    LocalEstimator(CANManager* can);

    /// @brief step through each motor and add to micro state
    /// @param output entire micro state 
    /// @param curr_state current micro state
    /// @param override override flag
    void step_states(float output[CAN_MAX_MOTORS][MICRO_STATE_LEN], float curr_state[CAN_MAX_MOTORS][MICRO_STATE_LEN], int override);
};

#endif
