
#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include "../comms/rm_can.hpp"
#include "state.hpp"
#include "../sensors/RefSystem.hpp"

#define NUM_SENSOR_VALUES 8

struct Estimator
{
public:
    Estimator(){};

    // Virtual so they don't ever get called over real estimators
    virtual void step_states(float outputs[STATE_LEN][3]);

    int get_num_states() { return num_states; }

    bool micro_estimator = false;

protected:
    /// @brief number of states that an estimator will estimate. 
    /// For the micro estimators its the number micro states to estimate
    int num_states;

    // create a timer object for each estimator
    Timer time;

    /// @brief Computes the magnitude of a vector given length n
    /// @param a Vector to compute the magnitude of
    /// @param n Length of Vector a
    /// @return returns the magnitude of a
    float __magnitude(float *a, int n)
    {
        float square_sum = 0;
        for (int i = 0; i < n; i++)
        {
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
    float __vectorProduct(float *a, float *b, int n)
    {
        float product = 0;
        for (int i = 0; i < n; i++)
        {
            product += a[i] * b[i];
        }
        return product;
    }

    /// @brief Computes the cross product of 2 given vectors of length 3
    /// @param v_A Vector A (3x1)
    /// @param v_B Vector B (3x1)
    /// @param output Cross product output vector (3x1)
    void __crossProduct(float v_A[], float v_B[], float output[])
    {
        output[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
        output[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
        output[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
    }

    /// @brief Rotates input_vector around the given unit_vector by theta radians
    /// @param unit_vector Vector to rotate around
    /// @param input_vector Vector to be rotated
    /// @param theta Angle to rotate (Rad)
    /// @param output New rotated vector
    void __rotateVector3D(float unit_vector[], float input_vector[], float theta, float output[])
    {
        float unit_cross_input[3];
        __crossProduct(unit_vector, input_vector, unit_cross_input);
        output[0] = (input_vector[0] * cos(theta)) + (unit_cross_input[0] * sin(theta)) + (unit_vector[0] * __vectorProduct(unit_vector, input_vector, 3) * (1 - cos(theta)));
        output[1] = (input_vector[1] * cos(theta)) + (unit_cross_input[1] * sin(theta)) + (unit_vector[1] * __vectorProduct(unit_vector, input_vector, 3) * (1 - cos(theta)));
        output[2] = (input_vector[2] * cos(theta)) + (unit_cross_input[2] * sin(theta)) + (unit_vector[2] * __vectorProduct(unit_vector, input_vector, 3) * (1 - cos(theta)));
    }

    /// @brief This functions finds the determinant of a 3x3 Matrix
    float determinantOfMatrix(float mat[3][3])
    {
        float ans;
        ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
        return ans;
    }

    /// @brief This function finds the solution of a 3x3 system of linear equations using cramer's rule.
    /// @param coeff 3x3 coeff matrix for the system with 3x1 solution matrix added to the end
    /// @param output 3x1 Array for the solutions
    void solveSystem(float coeff[3][4], float output[3])
    {
        // Matrix d using coeff as given in cramer's rule
        float d[3][3] = {{coeff[0][0], coeff[0][1], coeff[0][2]},
                          {coeff[1][0], coeff[1][1], coeff[1][2]},
                          {coeff[2][0], coeff[2][1], coeff[2][2]}};
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
            }
            else if (D1 != 0 || D2 != 0 || D3 != 0) {
                Serial.println("matrix solve bad2");
                output[0] = 0;
                output[1] = 0;
                output[2] = 0;
            }
        }
    }
};

struct GimbalEstimator : public Estimator
{
private:
    float YAW_ENCODER_OFFSET; // input variables
    float PITCH_ENCODER_OFFSET;
    float pitch_angle;
    float yaw_angle;
    float roll_angle;
    float chassis_angle;
    float chasis_pitch_angle;
    float gravity_accel_vector[3];
    float gravity_pitch_angle;
    float yaw_axis_spherical[3];
    float pitch_axis_spherical[3];
    float roll_axis_spherical[3];
    float yaw_axis_unitvector[3];
    float pitch_axis_unitvector[3];
    float roll_axis_unitvector[3];
    float yaw_axis_global[3];
    float pitch_axis_global[3];
    float roll_axis_global[3];
    float current_yaw_velocity = 0;
    float previous_yaw_velocity = 0;
    float current_pitch_velocity = 0;
    float previous_pitch_velocity = 0;
    float current_roll_velocity = 0;
    float previous_roll_velocity = 0;
    float global_yaw_velocity = 0;
    float global_roll_velocity = 0;
    float global_pitch_velocity = 0;
    float global_pitch_angle = 1.92;
    float global_yaw_angle = 0;
    float global_roll_angle = 0;
    float dt = 0;
    BuffEncoder *buff_enc_yaw;
    BuffEncoder *buff_enc_pitch;
    CANData *can_data;
    ICM20649 *icm_imu;

    float pos_estimate[3] = {0,0,0};

public:
    GimbalEstimator(float sensor_values[10], BuffEncoder *b1, BuffEncoder *b2, ICM20649 *imu, CANData *data, int n)
    {
        buff_enc_yaw = b1; // sensor object definitions
        buff_enc_pitch = b2;
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

    void step_states(float output[STATE_LEN][3]) override
    {
        float pitch_enc_angle = (-buff_enc_pitch->get_angle()) + PITCH_ENCODER_OFFSET;
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

        // gimbal rotation axis in spherical coordinates in imu refrence frame
        if (gravity_accel_vector[0] == 0)
            yaw_axis_spherical[1] = 1.57;
        else if (gravity_accel_vector[0] < 0)
        {
            yaw_axis_spherical[1] = PI + atan(gravity_accel_vector[1] / gravity_accel_vector[0]); // theta
        }
        else
        {
            yaw_axis_spherical[1] = atan(gravity_accel_vector[1] / gravity_accel_vector[0]); // theta
        }
        yaw_axis_spherical[2] = acos(gravity_accel_vector[2] / __magnitude(gravity_accel_vector, 3)) + pitch_diff; // phi

        // roll_axis_spherical[1] = yaw_axis_spherical[1]; // theta
        // roll_axis_spherical[2] = yaw_axis_spherical[2]-(PI*0.5); // phi

        // pitch_axis_spherical[1] = yaw_axis_spherical[1]-(PI*0.5);
        // pitch_axis_spherical[2] = (PI*0.5);

        // convert spherical to cartesian, These unit vectors are axis in the gimbal refrence frame
        yaw_axis_unitvector[0] = yaw_axis_spherical[0] * cos(yaw_axis_spherical[1]) * sin(yaw_axis_spherical[2]);
        yaw_axis_unitvector[1] = yaw_axis_spherical[0] * sin(yaw_axis_spherical[1]) * sin(yaw_axis_spherical[2]);
        yaw_axis_unitvector[2] = yaw_axis_spherical[0] * cos(yaw_axis_spherical[2]);

        // roll_axis_unitvector[0] = roll_axis_spherical[0]*cos(roll_axis_spherical[1])*sin(roll_axis_spherical[2]);
        // roll_axis_unitvector[1] = roll_axis_spherical[0]*sin(roll_axis_spherical[1])*sin(roll_axis_spherical[2]);
        // roll_axis_unitvector[2] = roll_axis_spherical[0]*cos(roll_axis_spherical[2]);

        // pitch_axis_unitvector[0] = pitch_axis_spherical[0]*cos(pitch_axis_spherical[1])*sin(pitch_axis_spherical[2]);
        // pitch_axis_unitvector[1] = pitch_axis_spherical[0]*sin(pitch_axis_spherical[1])*sin(pitch_axis_spherical[2]);
        // pitch_axis_unitvector[2] = pitch_axis_spherical[0]*cos(pitch_axis_spherical[2]);
        float mag = sqrt((-2.889366 * -2.889366) + (-0.026573 * -0.026573) + (-0.023502 * -0.023502));
        pitch_axis_unitvector[0] = -2.889366 / mag;
        pitch_axis_unitvector[1] = -0.026573 / mag;
        pitch_axis_unitvector[2] = -0.023502 / mag;

        __crossProduct(pitch_axis_unitvector, yaw_axis_unitvector, roll_axis_unitvector);

        float magicNum = 0.005; // left yaw increases with 0.8
        __rotateVector3D(roll_axis_unitvector, yaw_axis_unitvector, magicNum, yaw_axis_unitvector);
        __rotateVector3D(roll_axis_unitvector, pitch_axis_unitvector, magicNum, pitch_axis_unitvector);

        // __rotateVector3D(yaw_axis_unitvector,roll_axis_unitvector,(PI*0.5),pitch_axis_unitvector);

        // offset the axis' based on the pitch yaw and roll data, These vectors give global pitch yaw and roll
        __rotateVector3D(roll_axis_unitvector, yaw_axis_unitvector, global_roll_angle, yaw_axis_global);
        __rotateVector3D(roll_axis_unitvector, pitch_axis_unitvector, global_roll_angle, pitch_axis_global);

        __rotateVector3D(pitch_axis_unitvector, yaw_axis_unitvector, (global_pitch_angle - pitch_enc_angle), yaw_axis_global);
        __rotateVector3D(pitch_axis_unitvector, roll_axis_unitvector, (global_pitch_angle - pitch_enc_angle), roll_axis_global);

        // gets the velocity data from the imu and uses the gravity vector to calculate the yaw velocity
        float raw_omega_vector[3] = {icm_imu->get_gyro_X(), icm_imu->get_gyro_Y(), icm_imu->get_gyro_Z()};
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

        float imu_vel_offset = 8.05;
        // calculate the pitch yaw and roll velocities (Gimbal Relative)
        current_pitch_velocity = __vectorProduct(pitch_axis_unitvector, raw_omega_vector, 3)/imu_vel_offset;
        current_yaw_velocity = __vectorProduct(yaw_axis_unitvector, raw_omega_vector, 3)/imu_vel_offset;
        current_roll_velocity = __vectorProduct(roll_axis_unitvector, raw_omega_vector, 3)/imu_vel_offset;

        // calculate the pitch yaw and roll velocities (Global Reference)
        global_pitch_velocity = __vectorProduct(pitch_axis_global, raw_omega_vector, 3);
        global_yaw_velocity = __vectorProduct(yaw_axis_global, raw_omega_vector, 3);
        global_roll_velocity = __vectorProduct(roll_axis_global, raw_omega_vector, 3);
        // position integration
        dt = time.delta();
        if (dt > .002)
            dt = 0; // first dt loop generates huge time so check for that
        yaw_angle += -current_yaw_velocity * (dt);
        pitch_angle += current_pitch_velocity * (dt);
        roll_angle += -current_roll_velocity * (dt);

        global_yaw_angle += -global_yaw_velocity * (dt);
        global_pitch_angle += -global_pitch_velocity * (dt);
        global_roll_angle += -global_roll_velocity * (dt);

        chassis_angle = yaw_angle - yaw_enc_angle;
        // -chassis_angle

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
        output[3][2] = 0;
        output[4][0] = pitch_angle;
        output[4][1] = current_pitch_velocity;
        output[4][2] = pitch_enc_angle;

        // chassis estimation
        float front_right = can_data->get_motor_attribute(CAN_1, 1, MotorAttribute::SPEED);
        float back_right = can_data->get_motor_attribute(CAN_1, 2, MotorAttribute::SPEED);
        float back_left = can_data->get_motor_attribute(CAN_1, 3, MotorAttribute::SPEED);
        float front_left = can_data->get_motor_attribute(CAN_1, 4, MotorAttribute::SPEED);

        // m/s of chassis to motor rpm
        float x_scale = ((1/(PI*2*0.0516))*60)/0.10897435897;
        float y_scale = ((1/(PI*2*0.0516))*60)/0.10897435897;
        // chassis rad/s to motor rpm
        float psi_scale = ((.1835/(PI*2*0.0516))*60)/0.10897435897;
        // define coeff matracies for each system we want to solve
        float coeff_matrix1[3][4] = {{x_scale,0,psi_scale,front_right},{0,-y_scale,psi_scale,back_right},{-x_scale,0,psi_scale,back_left}};
        float coeff_matrix2[3][4] = {{x_scale,0,psi_scale,front_right},{0,-y_scale,psi_scale,back_right},{0,y_scale,psi_scale,front_left}};
        float coeff_matrix3[3][4] = {{x_scale,0,psi_scale,front_right},{0,y_scale,psi_scale,front_left},{-x_scale,0,psi_scale,back_left}};
        float coeff_matrix4[3][4] = {{0,-y_scale,psi_scale,back_right},{0,y_scale,psi_scale,front_left},{-x_scale,0,psi_scale,back_left}};

        // 4 solution sets of x, y, psi
        float vel_solutions[4][3];
        solveSystem(coeff_matrix1,vel_solutions[0]);
        solveSystem(coeff_matrix2,vel_solutions[1]);
        solveSystem(coeff_matrix3,vel_solutions[2]);
        solveSystem(coeff_matrix4,vel_solutions[3]);

        float vel_estimate[3];

        vel_estimate[0] = (cos(yaw_enc_angle-yaw_angle) * vel_solutions[0][0] + sin(yaw_enc_angle-yaw_angle) * vel_solutions[0][1]);
        vel_estimate[1] = (-sin(yaw_enc_angle-yaw_angle) * vel_solutions[0][0] + cos(yaw_enc_angle-yaw_angle) * vel_solutions[0][1]);
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
    
struct FlyWheelEstimator : public Estimator
{
    private:
     CANData* can_data;
     float projectile_speed_ref;
     float linear_velocity;

     float can_weight;
     float ref_weight;

    public:
    FlyWheelEstimator(CANData *c, int _num_states){
        can_data = c;
        num_states = _num_states;
    }

    void step_states(float output[STATE_LEN][3]){
        //can
        float radius = 30 * 0.001; //meters
        float angular_velocity_l = can_data->get_motor_attribute(CAN_2, 3, MotorAttribute::SPEED) / 60;
        float angular_velocity_r = can_data->get_motor_attribute(CAN_2, 4, MotorAttribute::SPEED) / 60;
        float angular_velocity_avg = (angular_velocity_l + angular_velocity_r)/2;
        linear_velocity = angular_velocity_avg * radius; //m/s

        //ref
        projectile_speed_ref = ref.ref_data.launching_event.projectile_initial_speed;

        output[0][1] = (projectile_speed_ref * ref_weight) + (linear_velocity * can_weight);
    }
};

struct FeederEstimator : public Estimator
{
    private:
        CANData* can_data;
        float balls_per_second_ref;
        float balls_per_second_can;

        float ref_weight;
        float can_weight;


    public:
    FeederEstimator(CANData *c, int _num_states){
        can_data = c;
        num_states = _num_states;
    }

    void step_states(float output[STATE_LEN][3]){
        //can
        float angular_velocity_motor = can_data->get_motor_attribute(CAN_2, 5, MotorAttribute::SPEED) / 60;
        float angular_velocity_feeder = angular_velocity_motor / 36;
        balls_per_second_can = angular_velocity_feeder * 8;

        //ref
        balls_per_second_ref = ref.ref_data.launching_event.launching_speed;

        output[0][1] = (balls_per_second_ref * ref_weight) * (balls_per_second_can * can_weight);
    }
};


struct LocalEstimator : public Estimator{
    private:
        CANData* can_data;


    public:
        LocalEstimator(CANData* c, int ns){
            micro_estimator = true;
            can_data = c;
            num_states = ns;
        }

        void step_states(float output[NUM_MOTORS][MICRO_STATE_LEN]){
            for (int i = 0; i < NUM_CAN_BUSES; i++){
                for(int j = 0; j < NUM_MOTORS_PER_BUS; j++){
                    output[(i*NUM_MOTORS_PER_BUS)+j][0] = (can_data->get_motor_attribute(i, j+1, MotorAttribute::SPEED)/60) * 2 * PI;
                }
            }
        }
};

#endif
