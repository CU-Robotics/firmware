#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include "../comms/rm_can.hpp"
#include "state.hpp"
#include "../sensors/RefSystem.hpp"

#define NUM_SENSOR_VALUES 8

/// @brief Parent estimator struct. All estimators should inherit from this.
struct Estimator {
public:
    Estimator() {};

    virtual ~Estimator() {};

    /// @brief step the current state(s) and update the estimate array accordingly
    /// @param outputs estimated state array to update with certain estimated states
    virtual void step_states(float outputs[STATE_LEN][3]);

    /// @brief gets the number of states that an estimator is estimating
    /// @return get number of states estimated by this estimator
    int get_num_states() { return num_states; }

    /// @brief bool that indicates if the estimator is a micro or macro estimator
    bool micro_estimator = false;

protected:
    /// @brief number of states that an estimator will estimate. For the micro estimators its the number micro states to estimate
    int num_states;

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
    void __crossProduct(float v_A[], float v_B[], float output[]) {
        output[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
        output[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
        output[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
    }

    /// @brief Rotates input_vector around the given unit_vector by theta radians
    /// @param unit_vector Vector to rotate around
    /// @param input_vector Vector to be rotated
    /// @param theta Angle to rotate (Rad)
    /// @param output New rotated vector
    void __rotateVector3D(float unit_vector[], float input_vector[], float theta, float output[]) {
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
                Serial.println("matrix solve bad1");
                output[0] = 0;
                output[1] = 0;
                output[2] = 0;
            } else if (D1 != 0 || D2 != 0 || D3 != 0) {
                Serial.println("matrix solve bad2");
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
    /// @brief calculated chassis pitch angle
    float chasis_pitch_angle;
    /// @brief gravity acceleration vector
    float gravity_accel_vector[3];
    /// @brief gravity pitch angle
    float gravity_pitch_angle;
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
    float curr_rev_raw[3] = {0};
    /// @brief previous rev encoder raw value
    float prev_rev_raw[3] = {0};
    /// @brief total radians travelled by each rev encoder
    float total_rev[3] = {0};
    /// @brief rev encoder difference
    float rev_diff[3] = {0};
    /// @brief delta time
    float dt = 0;

    /// @brief buff encoder on the yaw
    BuffEncoder* buff_enc_yaw;
    /// @brief buff encoder on the pitch
    BuffEncoder* buff_enc_pitch;
    /// @brief Odom encoder 1
    RevEncoder* rev_enc[3];
    /// @brief can data pointer from EstimatorManager
    CANData* can_data;
    /// @brief icm imu
    ICM20649* icm_imu;

    /// @brief position estimate to store position after integrating used for chassis odometry
    float pos_estimate[3] = { 0,0,0 };

public:
    /// @brief estimate the state of the gimbal
    /// @param sensor_values inputted sensor values from EstimatorManager
    /// @param b1 buff encoder 1
    /// @param b2 buff encoder 2
    /// @param imu icm encoder
    /// @param data can data from Estimator Manager
    /// @param n num states this estimator estimates
    GimbalEstimator(float sensor_values[10],RevEncoder* r1, RevEncoder* r2, RevEncoder* r3, BuffEncoder* b1, BuffEncoder* b2, ICM20649* imu, CANData* data, int n) {
        buff_enc_yaw = b1; // sensor object definitions
        buff_enc_pitch = b2;
        rev_enc[0] = r1;
        rev_enc[1] = r2;
        rev_enc[2] = r3;
        can_data = data;
        icm_imu = imu;
        num_states = n; // number of estimated states
        PITCH_ENCODER_OFFSET = sensor_values[1];
        YAW_ENCODER_OFFSET = sensor_values[0];
        pitch_angle = sensor_values[3];
        yaw_angle = sensor_values[2];
        roll_angle = sensor_values[4];
        chasis_pitch_angle = sensor_values[5];
        chassis_angle = 0;
        gravity_accel_vector[0] = sensor_values[6];
        gravity_accel_vector[1] = sensor_values[7];
        gravity_accel_vector[2] = sensor_values[8];
        gravity_pitch_angle = sensor_values[9];
        // definitions for spherical coordinates of new axis in the imu refrence frame
        yaw_axis_spherical[0] = 1;   // rho (1 for a spherical)
        pitch_axis_spherical[0] = 1; // rho (1 for a spherical)
        roll_axis_spherical[0] = 1;  // rho (1 for a spherical)
    }

    ~GimbalEstimator() {};
  
    /// @brief calculate estimated states and add to output array
    /// @param output output array to add estimated states to
    void step_states(float output[STATE_LEN][3]) override {
        float pitch_enc_angle = (-buff_enc_pitch->get_angle()) - PITCH_ENCODER_OFFSET;
        while (pitch_enc_angle >= PI)
            pitch_enc_angle -= 2 * PI;
        while (pitch_enc_angle <= -PI)
            pitch_enc_angle += 2 * PI;

        float yaw_enc_angle = (buff_enc_yaw->get_angle()) - YAW_ENCODER_OFFSET;
        while (yaw_enc_angle >= PI)
            yaw_enc_angle -= 2 * PI;
        while (yaw_enc_angle <= -PI)
            yaw_enc_angle += 2 * PI;

        // calculates yaw velocity before integrating to find position
        // calculates the difference in initial and current pitch angle
        float pitch_diff = gravity_pitch_angle - pitch_enc_angle;

        // Serial.println(pitch_enc_angle);

        // gimbal rotation axis in spherical coordinates in imu refrence frame
        if (gravity_accel_vector[0] == 0)
            yaw_axis_spherical[1] = 1.57;
        else if (gravity_accel_vector[0] < 0) {
            yaw_axis_spherical[1] = PI + atan(gravity_accel_vector[1] / gravity_accel_vector[0]); // theta
        } else {
            yaw_axis_spherical[1] = atan(gravity_accel_vector[1] / gravity_accel_vector[0]); // theta
        }
        yaw_axis_spherical[2] = acos(gravity_accel_vector[2] / __magnitude(gravity_accel_vector, 3)) - pitch_diff; // phi

        // roll_axis_spherical[1] = yaw_axis_spherical[1]; // theta
        // roll_axis_spherical[2] = yaw_axis_spherical[2]-(PI*0.5); // phi

        // pitch_axis_spherical[1] = yaw_axis_spherical[1]-(PI*0.5);
        // pitch_axis_spherical[2] = (PI*0.5);

        // convert spherical to cartesian, These unit vectors are axis in the gimbal refrence frame
        yaw_axis_unitvector[0] = yaw_axis_spherical[0] * cos(yaw_axis_spherical[1]) * sin(yaw_axis_spherical[2]);
        yaw_axis_unitvector[1] = yaw_axis_spherical[0] * sin(yaw_axis_spherical[1]) * sin(yaw_axis_spherical[2]);
        yaw_axis_unitvector[2] = yaw_axis_spherical[0] * cos(yaw_axis_spherical[2]);

        // Serial.printf("x: %f,y: %f,z: %f\n", roll_axis_unitvector[0],roll_axis_unitvector[1],roll_axis_unitvector[2]);

        // roll_axis_unitvector[0] = roll_axis_spherical[0]*cos(roll_axis_spherical[1])*sin(roll_axis_spherical[2]);
        // roll_axis_unitvector[1] = roll_axis_spherical[0]*sin(roll_axis_spherical[1])*sin(roll_axis_spherical[2]);
        // roll_axis_unitvector[2] = roll_axis_spherical[0]*cos(roll_axis_spherical[2]);

        // pitch_axis_unitvector[0] = pitch_axis_spherical[0]*cos(pitch_axis_spherical[1])*sin(pitch_axis_spherical[2]);
        // pitch_axis_unitvector[1] = pitch_axis_spherical[0]*sin(pitch_axis_spherical[1])*sin(pitch_axis_spherical[2]);
        // pitch_axis_unitvector[2] = pitch_axis_spherical[0]*cos(pitch_axis_spherical[2]);
        float mag = sqrt((2.789852 * 2.789852) + (0.045252 * 0.045252) + (0.009307 * 0.009307));
        pitch_axis_unitvector[0] = 2.789852 / mag;
        pitch_axis_unitvector[1] = 0.045252 / mag;
        pitch_axis_unitvector[2] = 0.009307 / mag;

        __crossProduct(pitch_axis_unitvector, yaw_axis_unitvector, roll_axis_unitvector);

        float magicNum = 0; // left yaw increases with 0.8
        __rotateVector3D(roll_axis_unitvector, yaw_axis_unitvector, magicNum, yaw_axis_unitvector);
        __rotateVector3D(roll_axis_unitvector, pitch_axis_unitvector, magicNum, pitch_axis_unitvector);

        // __rotateVector3D(yaw_axis_unitvector,roll_axis_unitvector,(PI*0.5),pitch_axis_unitvector);

        // offset the axis' based on the pitch yaw and roll data, These vectors give global pitch yaw and roll
        // __rotateVector3D(roll_axis_unitvector, yaw_axis_unitvector, global_roll_angle, yaw_axis_global);
        // __rotateVector3D(roll_axis_unitvector, pitch_axis_unitvector, global_roll_angle, pitch_axis_global);

        // __rotateVector3D(pitch_axis_unitvector, yaw_axis_unitvector, (global_pitch_angle - pitch_enc_angle), yaw_axis_global);
        // __rotateVector3D(pitch_axis_unitvector, roll_axis_unitvector, (global_pitch_angle - pitch_enc_angle), roll_axis_global);

        // gets the velocity data from the imu and uses the gravity vector to calculate the yaw velocity
        float raw_omega_vector[3] = { icm_imu->get_gyro_X(), icm_imu->get_gyro_Y(), icm_imu->get_gyro_Z() };
        // *Note: X is pitch Y is Roll Z is Yaw, when level
        // positive pitch angle is up, positive roll angle is right(robot pov), positive yaw is left(robot pov)

        float temp1[3];
        float temp2[3];
        float temp3[3];
        __crossProduct(yaw_axis_unitvector, pitch_axis_unitvector, temp1);
        __crossProduct(yaw_axis_unitvector, roll_axis_unitvector, temp2);
        __crossProduct(roll_axis_unitvector, pitch_axis_unitvector, temp3);

        // update previous to the current value before current is updated
        previous_pitch_velocity = current_pitch_velocity;
        previous_yaw_velocity = current_yaw_velocity;
        previous_roll_velocity = current_roll_velocity;

        float imu_vel_offset = 1;
        // calculate the pitch yaw and roll velocities (Gimbal Relative)
        current_pitch_velocity = __vectorProduct(pitch_axis_unitvector, raw_omega_vector, 3) / imu_vel_offset;
        current_yaw_velocity = __vectorProduct(yaw_axis_unitvector, raw_omega_vector, 3) / imu_vel_offset;
        current_roll_velocity = -__vectorProduct(roll_axis_unitvector, raw_omega_vector, 3) / imu_vel_offset;

        // calculate the pitch yaw and roll velocities (Global Reference)
        global_pitch_velocity = __vectorProduct(pitch_axis_global, raw_omega_vector, 3);
        global_yaw_velocity = __vectorProduct(yaw_axis_global, raw_omega_vector, 3);
        global_roll_velocity = __vectorProduct(roll_axis_global, raw_omega_vector, 3);
        // position integration
        dt = time.delta();
        if (dt > .002)
            dt = 0; // first dt loop generates huge time so check for that
        yaw_angle += current_yaw_velocity * (dt);
        pitch_angle += current_pitch_velocity * (dt);
        roll_angle += current_roll_velocity * (dt);

        global_yaw_angle += -global_yaw_velocity * (dt);
        global_pitch_angle += -global_pitch_velocity * (dt);
        global_roll_angle += -global_roll_velocity * (dt);

        // chassis_angle = yaw_angle - yaw_enc_angle;
        chassis_angle = -yaw_enc_angle;


        while (yaw_angle >= PI)
            yaw_angle -= 2 * PI;
        while (yaw_angle <= -PI)
            yaw_angle += 2 * PI;

        while (chassis_angle >= PI)
            chassis_angle -= 2 * PI;
        while (chassis_angle <= -PI)
            chassis_angle += 2 * PI;

        output[2][0] = chassis_angle;
        // output[2][1] = 0;
        // output[2][2] = 0;
        output[3][0] = yaw_angle;
        output[3][1] = current_yaw_velocity;
        output[3][2] = roll_angle;
        output[4][0] = pitch_enc_angle;
        output[4][1] = current_pitch_velocity;
        output[4][2] = pitch_enc_angle;

        for(int i = 0; i < 3; i++){
            curr_rev_raw[i] = rev_enc[i]->get_angle_radians();
            if ((curr_rev_raw[i]-prev_rev_raw[i]) > PI) rev_diff[i] = ((curr_rev_raw[i]-prev_rev_raw[i])-(2*PI));
            else if ((curr_rev_raw[i]-prev_rev_raw[i]) < -PI) rev_diff[i] = ((curr_rev_raw[i]-prev_rev_raw[i])+(2*PI));
            else rev_diff[i] = (curr_rev_raw[i]-prev_rev_raw[i]);
            total_rev[i] = rev_diff[i]+total_rev[i];
            prev_rev_raw[i] = curr_rev_raw[i];
        }

        // chassis estimation
        float front_right = can_data->get_motor_attribute(CAN_1, 1, MotorAttribute::SPEED);
        float back_right = can_data->get_motor_attribute(CAN_1, 2, MotorAttribute::SPEED);
        float back_left = can_data->get_motor_attribute(CAN_1, 3, MotorAttribute::SPEED);
        float front_left = can_data->get_motor_attribute(CAN_1, 4, MotorAttribute::SPEED);

        // m/s of chassis to motor rpm
        float x_scale = ((1 / (PI * 2 * 0.0516)) * 60) / 0.10897435897;
        float y_scale = ((1 / (PI * 2 * 0.0516)) * 60) / 0.10897435897;
        // chassis rad/s to motor rpm
        float psi_scale = ((.1835 / (PI * 2 * 0.0516)) * 60) / 0.10897435897;
        // define coeff matracies for each system we want to solve
        float coeff_matrix1[3][4] = { {x_scale,0,psi_scale,front_right},{0,-y_scale,psi_scale,back_right},{-x_scale,0,psi_scale,back_left} };
        float coeff_matrix2[3][4] = { {x_scale,0,psi_scale,front_right},{0,-y_scale,psi_scale,back_right},{0,y_scale,psi_scale,front_left} };
        float coeff_matrix3[3][4] = { {x_scale,0,psi_scale,front_right},{0,y_scale,psi_scale,front_left},{-x_scale,0,psi_scale,back_left} };
        float coeff_matrix4[3][4] = { {0,-y_scale,psi_scale,back_right},{0,y_scale,psi_scale,front_left},{-x_scale,0,psi_scale,back_left} };

        // 4 solution sets of x, y, psi
        float vel_solutions[4][3];
        solveSystem(coeff_matrix1, vel_solutions[0]);
        solveSystem(coeff_matrix2, vel_solutions[1]);
        solveSystem(coeff_matrix3, vel_solutions[2]);
        solveSystem(coeff_matrix4, vel_solutions[3]);

        float vel_estimate[3];

        vel_estimate[0] = (cos(yaw_enc_angle - yaw_angle) * vel_solutions[0][0] + sin(yaw_enc_angle - yaw_angle) * vel_solutions[0][1]);
        vel_estimate[1] = (-sin(yaw_enc_angle - yaw_angle) * vel_solutions[0][0] + cos(yaw_enc_angle - yaw_angle) * vel_solutions[0][1]);
        vel_estimate[2] = vel_solutions[0][2];

        // integrate to find pos
        pos_estimate[0] += vel_estimate[0] * dt;
        pos_estimate[1] += vel_estimate[1] * dt;
        pos_estimate[2] += vel_estimate[2] * dt;

        output[0][0] = pos_estimate[0]; // x pos
        output[0][1] = vel_estimate[0];
        output[0][2] = 0;
        output[1][0] = pos_estimate[1]; // y pos
        output[1][1] = vel_estimate[1];
        output[1][2] = 0;
        // output[2][0] = 0; // chassis angle
        output[2][1] = vel_estimate[2];
        output[2][2] = yaw_enc_angle;
    }
};

/// @brief Estimate the state of the flywheels as meters/second of balls exiting the barrel.
struct FlyWheelEstimator : public Estimator {
private:
    /// @brief can data pointer from EstimatorManager
    CANData* can_data;
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
    /// @param c can data pointer from EstimatorManager
    /// @param _num_states number of states estimated
    FlyWheelEstimator(CANData* c, int _num_states) {
        can_data = c;
        num_states = _num_states;
    }
  
    ~FlyWheelEstimator() {};

    /// @brief generate estimated states and replace in output array
    /// @param output array to be updated with the calculated states
    void step_states(float output[STATE_LEN][3]) {
        //can
        float radius = 30 * 0.001; //meters
        float angular_velocity_l = -can_data->get_motor_attribute(CAN_2, 3, MotorAttribute::SPEED) * (2 * PI) / 60;
        float angular_velocity_r = can_data->get_motor_attribute(CAN_2, 4, MotorAttribute::SPEED) * (2 * PI) / 60;
        float angular_velocity_avg = (angular_velocity_l + angular_velocity_r) / 2;
        linear_velocity = angular_velocity_avg * radius; //m/s

        //ref
        projectile_speed_ref = ref.ref_data.launching_event.projectile_initial_speed;

        //weighted average
        output[0][1] = (projectile_speed_ref * ref_weight) + (linear_velocity * can_weight);
    }
};

/// @brief Estimate the state of the feeder in balls/s
struct FeederEstimator : public Estimator {
private:
    /// @brief can data pointer from EstimatorManager
    CANData* can_data;

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
    /// @param c can data pointer from EstimatorManager
    /// @param _num_states number of states this estimator estimates
    FeederEstimator(CANData* c, int _num_states) {
        can_data = c;
        num_states = _num_states;
    }

    ~FeederEstimator() {};

    /// @brief calculate state updates
    /// @param output updated balls per second of feeder
    void step_states(float output[STATE_LEN][3]) {
        //can
        float angular_velocity_motor = can_data->get_motor_attribute(CAN_2, 5, MotorAttribute::SPEED) / 60;
        float angular_velocity_feeder = angular_velocity_motor / 36;
        balls_per_second_can = angular_velocity_feeder * 8;

        //ref
        balls_per_second_ref = ref.ref_data.launching_event.launching_speed;

        output[0][1] = (balls_per_second_ref * ref_weight) + (balls_per_second_can * can_weight);

    }
};

/// @brief Estimate the state of the switcher in millimeters
struct SwitcherEstimator : public Estimator {
private:
    /// @brief can data pointer from EstimatorManager
    CANData* can_data;

    /// @brief TOF sensor pointer from EstimatorManager
    TOFSensor* time_of_flight;

    /// @brief can weight for weighted average
    float tof_sensor_offset = 0;

    /// @brief used to scale the tof sensor data to -1 to 1
    float tof_scale = 0;
public:
    /// @brief make new barrel switcher estimator and set can_data pointer and num_states
    /// @param c can data pointer from EstimatorManager
    /// @param _num_states number of states this estimator estimates
    /// @param tof time of flight sensor object
    /// @param values array of values to set tof sensor offset and scale
    SwitcherEstimator(float values[2],CANData* c,TOFSensor* tof, int _num_states) {
        can_data = c;
        num_states = _num_states;
        time_of_flight = tof;
        tof_sensor_offset = values[0];
        tof_scale = values[1];
    }

    /// @brief calculate state updates
    /// @param output updated balls per second of feeder
    void step_states(float output[STATE_LEN][3]) {
        //read tof sensor (millimeters)
        float distance_from_right = ((float)(time_of_flight->read()) - tof_sensor_offset)/tof_scale;
        float angular_velocity_motor = -((((can_data->get_motor_attribute(CAN_2, 6, MotorAttribute::SPEED) / 60)*(2*PI))/36.0)*(5.1))/tof_scale;
        output[0][0] = distance_from_right;
        output[0][1] = angular_velocity_motor;

    }
};

/// @brief This estimator estimates our "micro" state which is stores all the motor velocities(in rad/s), whereas the other estimators estimate "macro" state which stores robot joints
struct LocalEstimator : public Estimator {
private:
    /// @brief can data from EstimatorManager
    CANData* can_data;


public:
    /// @brief Make new local estimator and set can data pointer and num states
    /// @param c can data pointer from EstimatorManager
    /// @param ns number of states this estimator estimates
    LocalEstimator(CANData* c, int ns) {
        micro_estimator = true;
        can_data = c;
        num_states = ns;
    }

    /// @brief step through each motor and add to micro state
    /// @param output entire micro state 
    void step_states(float output[NUM_MOTORS][MICRO_STATE_LEN]) {
        for (int i = 0; i < NUM_CAN_BUSES; i++) {
            for (int j = 0; j < NUM_MOTORS_PER_BUS; j++) {
                output[(i * NUM_MOTORS_PER_BUS) + j][0] = (can_data->get_motor_attribute(i, j + 1, MotorAttribute::SPEED) / 60) * 2 * PI;
            }
        }
    }
};

#endif
