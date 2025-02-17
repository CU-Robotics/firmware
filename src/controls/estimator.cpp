#include "estimator.hpp"

GimbalEstimator::GimbalEstimator(Config config_data, SensorManager* sensor_manager, CANManager* _can) {
        buff_enc_yaw = sensor_manager->get_buff_encoder(0); // sensor object definitions
        buff_enc_pitch = sensor_manager->get_buff_encoder(1);
        rev_enc[0] = sensor_manager->get_rev_sensor(0);
        rev_enc[1] = sensor_manager->get_rev_sensor(1);
        rev_enc[2] = sensor_manager->get_rev_sensor(2);
        can = _can;
        icm_imu = sensor_manager->get_icm_sensor(0);
        YAW_ENCODER_OFFSET = config_data.sensor_info[0][2];
        PITCH_ENCODER_OFFSET = config_data.sensor_info[1][2];

    yaw_angle = config_data.sensor_info[2][2];
    pitch_angle = config_data.sensor_info[2][3];
    roll_angle = config_data.sensor_info[2][4];
    chassis_angle = 0;
    imu_yaw_axis_vector[0] = config_data.sensor_info[2][5];
    imu_yaw_axis_vector[1] = config_data.sensor_info[2][6];
    imu_yaw_axis_vector[2] = config_data.sensor_info[2][7];
    imu_pitch_axis_vector[0] = config_data.sensor_info[2][8];
    imu_pitch_axis_vector[1] = config_data.sensor_info[2][9];
    imu_pitch_axis_vector[2] = config_data.sensor_info[2][10];
    starting_pitch_angle = config_data.sensor_info[2][1];

    odom_wheel_radius = config_data.sensor_info[3][2];
    odom_axis_offset_x = config_data.sensor_info[3][1];
    odom_axis_offset_y = config_data.sensor_info[4][1];
    // definitions for spherical coordinates of new axis in the imu refrence frame
    yaw_axis_spherical[0] = 1;   // rho (1 for a spherical)
    pitch_axis_spherical[0] = 1; // rho (1 for a spherical)
    roll_axis_spherical[0] = 1;  // rho (1 for a spherical)
}

void GimbalEstimator::step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override) {
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
    float pitch_diff = starting_pitch_angle - pitch_enc_angle;

    // gimbal rotation axis in spherical coordinates in imu refrence frame
    if (imu_yaw_axis_vector[0] == 0)
        yaw_axis_spherical[1] = 1.57;
    else if (imu_yaw_axis_vector[0] < 0) {
        yaw_axis_spherical[1] = PI + atan(imu_yaw_axis_vector[1] / imu_yaw_axis_vector[0]); // theta
    } else {
        yaw_axis_spherical[1] = atan(imu_yaw_axis_vector[1] / imu_yaw_axis_vector[0]); // theta
    }
    yaw_axis_spherical[2] = acos(imu_yaw_axis_vector[2] / __magnitude(imu_yaw_axis_vector, 3)) - pitch_diff; // phi

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
    float mag = sqrt(pow(imu_pitch_axis_vector[0], 2) + pow(imu_pitch_axis_vector[1], 2) + pow(imu_pitch_axis_vector[2], 2));
    pitch_axis_unitvector[0] = imu_pitch_axis_vector[0] / mag;
    pitch_axis_unitvector[1] = imu_pitch_axis_vector[1] / mag;
    pitch_axis_unitvector[2] = imu_pitch_axis_vector[2] / mag;

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

    // chassis_angle = yaw_angle - yaw_enc_angle;
    chassis_angle = -yaw_enc_angle;
    if (count1 == 0) {
        initial_chassis_angle = chassis_angle;
        prev_chassis_angle = chassis_angle;
        count1++;
        dt = 0;
    }
    if (override == 1)yaw_angle = curr_state[3][0];
    yaw_angle += current_yaw_velocity * (dt);
    pitch_angle += current_pitch_velocity * (dt);
    roll_angle += current_roll_velocity * (dt);

    global_yaw_angle += -global_yaw_velocity * (dt);
    global_pitch_angle += -global_pitch_velocity * (dt);
    global_roll_angle += -global_roll_velocity * (dt);

    while (yaw_angle >= PI)
        yaw_angle -= 2 * PI;
    while (yaw_angle <= -PI)
        yaw_angle += 2 * PI;

    while (chassis_angle >= PI)
        chassis_angle -= 2 * PI;
    while (chassis_angle <= -PI)
        chassis_angle += 2 * PI;

    // output[2][0] = chassis_angle;
    // output[2][1] = 0;
    // output[2][2] = 0;
    output[3][0] = yaw_angle;
    output[3][1] = current_yaw_velocity;
    output[3][2] = roll_angle;
    output[4][0] = pitch_enc_angle;
    output[4][1] = current_pitch_velocity;
    output[4][2] = pitch_enc_angle;

    for (int i = 0; i < 3; i++) {
        curr_rev_raw[i] = rev_enc[i]->get_angle_radians();

        if ((curr_rev_raw[i] - prev_rev_raw[i]) > PI) rev_diff[i] = ((curr_rev_raw[i] - prev_rev_raw[i]) - (2 * PI));
        else if ((curr_rev_raw[i] - prev_rev_raw[i]) < -PI) rev_diff[i] = ((curr_rev_raw[i] - prev_rev_raw[i]) + (2 * PI));
        else rev_diff[i] = (curr_rev_raw[i] - prev_rev_raw[i]);
        prev_rev_raw[i] = curr_rev_raw[i];
        odom_pos_diff[i] = rev_diff[i] * odom_wheel_radius;
        total_odom_pos[i] = odom_pos_diff[i] + total_odom_pos[i];
    }


    chassis_angle = yaw_angle - yaw_enc_angle;
    // chassis_angle = -(total_odom_pos[0] + total_odom_pos[2])/(2*odom_axis_offset_x)+initial_chassis_angle;  
    float d_chassis_heading = (chassis_angle - prev_chassis_angle);
    if (d_chassis_heading > PI) d_chassis_heading -= 2 * PI;
    else if (d_chassis_heading < -PI) d_chassis_heading += 2 * PI;
    prev_chassis_angle = chassis_angle;
    if (override == 1) {
        pos_estimate[0] = curr_state[0][0];
        pos_estimate[1] = curr_state[1][0];
        previous_pos[0] = curr_state[0][0];
        previous_pos[1] = curr_state[1][0];
    }
    if (d_chassis_heading == 0) {
        pos_estimate[0] += ((odom_pos_diff[0]) * cos(chassis_angle + odom_angle_offset)) - ((odom_pos_diff[1]) * sin(chassis_angle + odom_angle_offset));
        pos_estimate[1] += ((odom_pos_diff[0]) * sin(chassis_angle + odom_angle_offset)) + ((odom_pos_diff[1]) * cos(chassis_angle + odom_angle_offset));
    } else {
        float pod1_relative = 2 * sin(d_chassis_heading * 0.5) * ((odom_pos_diff[0] / d_chassis_heading) + odom_axis_offset_x); //relative motion of the first pod
        float pod2_relative = 2 * sin(d_chassis_heading * 0.5) * ((odom_pos_diff[1] / d_chassis_heading) + odom_axis_offset_y); //relative motion of the second pod
        pos_estimate[0] += pod1_relative * cos((chassis_angle + odom_angle_offset) + (d_chassis_heading * 0.5))
            - pod2_relative * sin((chassis_angle + odom_angle_offset) + (d_chassis_heading * 0.5));
        pos_estimate[1] += pod1_relative * sin((chassis_angle + odom_angle_offset) + (d_chassis_heading * 0.5))
            + pod2_relative * cos((chassis_angle + odom_angle_offset) + (d_chassis_heading * 0.5));
    }

    // // chassis estimation
    // float front_right = can_data->get_motor_attribute(CAN_1, 1, MotorAttribute::SPEED);
    // float back_right = can_data->get_motor_attribute(CAN_1, 2, MotorAttribute::SPEED);
    // float back_left = can_data->get_motor_attribute(CAN_1, 3, MotorAttribute::SPEED);
    // float front_left = can_data->get_motor_attribute(CAN_1, 4, MotorAttribute::SPEED);

    // // m/s of chassis to motor rpm
    // float x_scale = ((1 / (PI * 2 * 0.0516)) * 60) / 0.10897435897;
    // float y_scale = ((1 / (PI * 2 * 0.0516)) * 60) / 0.10897435897;
    // // chassis rad/s to motor rpm
    // float psi_scale = ((.1835 / (PI * 2 * 0.0516)) * 60) / 0.10897435897;
    // // define coeff matracies for each system we want to solve
    // float coeff_matrix1[3][4] = { {x_scale,0,psi_scale,front_right},{0,-y_scale,psi_scale,back_right},{-x_scale,0,psi_scale,back_left} };
    // float coeff_matrix2[3][4] = { {x_scale,0,psi_scale,front_right},{0,-y_scale,psi_scale,back_right},{0,y_scale,psi_scale,front_left} };
    // float coeff_matrix3[3][4] = { {x_scale,0,psi_scale,front_right},{0,y_scale,psi_scale,front_left},{-x_scale,0,psi_scale,back_left} };
    // float coeff_matrix4[3][4] = { {0,-y_scale,psi_scale,back_right},{0,y_scale,psi_scale,front_left},{-x_scale,0,psi_scale,back_left} };

    // // 4 solution sets of x, y, psi
    // float vel_solutions[4][3];
    // solveSystem(coeff_matrix1, vel_solutions[0]);
    // solveSystem(coeff_matrix2, vel_solutions[1]);
    // solveSystem(coeff_matrix3, vel_solutions[2]);
    // solveSystem(coeff_matrix4, vel_solutions[3]);

    // float vel_estimate[3];

    // vel_estimate[0] = (cos(yaw_enc_angle - yaw_angle) * vel_solutions[0][0] + sin(yaw_enc_angle - yaw_angle) * vel_solutions[0][1]);
    // vel_estimate[1] = (-sin(yaw_enc_angle - yaw_angle) * vel_solutions[0][0] + cos(yaw_enc_angle - yaw_angle) * vel_solutions[0][1]);
    // vel_estimate[2] = vel_solutions[0][2];

    // integrate to find pos
    // pos_estimate[0] += vel_estimate[0] * dt;
    // pos_estimate[1] += vel_estimate[1] * dt;
    // pos_estimate[2] += vel_estimate[2] * dt;



    output[0][0] = pos_estimate[0]; // x pos
    output[0][1] = (pos_estimate[0] - previous_pos[0]) / dt;
    output[0][2] = 0;
    output[1][0] = pos_estimate[1]; // y pos
    output[1][1] = (pos_estimate[1] - previous_pos[1]) / dt;
    output[1][2] = 0;
    output[2][0] = chassis_angle; // chassis angle
    output[2][1] = d_chassis_heading / dt;
    output[2][2] = yaw_enc_angle;



    previous_pos[0] = pos_estimate[0];
    previous_pos[1] = pos_estimate[1];
}

GimbalEstimatorNoOdom::GimbalEstimatorNoOdom(Config config_data, SensorManager* sensor_manager, CANManager* _can) {
    buff_enc_yaw = sensor_manager->get_buff_encoder(0); // sensor object definitions
    buff_enc_pitch = sensor_manager->get_buff_encoder(1);
    can = _can;
    icm_imu = sensor_manager->get_icm_sensor(0);
    YAW_ENCODER_OFFSET = config_data.sensor_info[0][2];
    PITCH_ENCODER_OFFSET = config_data.sensor_info[1][2];

    yaw_angle = config_data.sensor_info[2][2];
    pitch_angle = config_data.sensor_info[2][3];
    roll_angle = config_data.sensor_info[2][4];
    chassis_angle = 0;
    imu_yaw_axis_vector[0] = config_data.sensor_info[2][5];
    imu_yaw_axis_vector[1] = config_data.sensor_info[2][6];
    imu_yaw_axis_vector[2] = config_data.sensor_info[2][7];
    imu_pitch_axis_vector[0] = config_data.sensor_info[2][8];
    imu_pitch_axis_vector[1] = config_data.sensor_info[2][9];
    imu_pitch_axis_vector[2] = config_data.sensor_info[2][10];
    starting_pitch_angle = config_data.sensor_info[2][1];

    odom_wheel_radius = config_data.sensor_info[3][2];
    odom_axis_offset_x = config_data.sensor_info[3][1];
    odom_axis_offset_y = config_data.sensor_info[4][1];
    // definitions for spherical coordinates of new axis in the imu refrence frame
    yaw_axis_spherical[0] = 1;   // rho (1 for a spherical)
    pitch_axis_spherical[0] = 1; // rho (1 for a spherical)
    roll_axis_spherical[0] = 1;  // rho (1 for a spherical)
}

void GimbalEstimatorNoOdom::step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override) {
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
    float pitch_diff = starting_pitch_angle - pitch_enc_angle;

    // gimbal rotation axis in spherical coordinates in imu refrence frame
    if (imu_yaw_axis_vector[0] == 0)
        yaw_axis_spherical[1] = 1.57;
    else if (imu_yaw_axis_vector[0] < 0) {
        yaw_axis_spherical[1] = PI + atan(imu_yaw_axis_vector[1] / imu_yaw_axis_vector[0]); // theta
    } else {
        yaw_axis_spherical[1] = atan(imu_yaw_axis_vector[1] / imu_yaw_axis_vector[0]); // theta
    }
    yaw_axis_spherical[2] = acos(imu_yaw_axis_vector[2] / __magnitude(imu_yaw_axis_vector, 3)) - pitch_diff; // phi

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
    float mag = sqrt(pow(imu_pitch_axis_vector[0], 2) + pow(imu_pitch_axis_vector[1], 2) + pow(imu_pitch_axis_vector[2], 2));
    pitch_axis_unitvector[0] = imu_pitch_axis_vector[0] / mag;
    pitch_axis_unitvector[1] = imu_pitch_axis_vector[1] / mag;
    pitch_axis_unitvector[2] = imu_pitch_axis_vector[2] / mag;

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
    if (dt > .1)
        dt = 0; // first dt loop generates huge time so check for that
    yaw_angle += current_yaw_velocity * (dt);
    pitch_angle += current_pitch_velocity * (dt);
    roll_angle += current_roll_velocity * (dt);

    global_yaw_angle += -global_yaw_velocity * (dt);
    global_pitch_angle += -global_pitch_velocity * (dt);
    global_roll_angle += -global_roll_velocity * (dt);

    // chassis_angle = yaw_angle - yaw_enc_angle;
    chassis_angle = -yaw_enc_angle;
    if (count1 == 0) {
        initial_chassis_angle = chassis_angle;
        prev_chassis_angle = chassis_angle;
        count1++;
    }

    while (yaw_angle >= PI)
        yaw_angle -= 2 * PI;
    while (yaw_angle <= -PI)
        yaw_angle += 2 * PI;

    while (chassis_angle >= PI)
        chassis_angle -= 2 * PI;
    while (chassis_angle <= -PI)
        chassis_angle += 2 * PI;

    // output[2][0] = chassis_angle;
    // output[2][1] = 0;
    // output[2][2] = 0;
    output[3][0] = yaw_angle;
    output[3][1] = current_yaw_velocity;
    output[3][2] = roll_angle;
    output[4][0] = pitch_enc_angle;
    output[4][1] = current_pitch_velocity;
    output[4][2] = pitch_enc_angle;


    // // chassis estimation
    // float front_right = can_data->get_motor_attribute(CAN_1, 1, MotorAttribute::SPEED);
    // float back_right = can_data->get_motor_attribute(CAN_1, 2, MotorAttribute::SPEED);
    // float back_left = can_data->get_motor_attribute(CAN_1, 3, MotorAttribute::SPEED);
    // float front_left = can_data->get_motor_attribute(CAN_1, 4, MotorAttribute::SPEED);

    // // m/s of chassis to motor rpm
    // float x_scale = ((1 / (PI * 2 * 0.0516)) * 60) / 0.10897435897;
    // float y_scale = ((1 / (PI * 2 * 0.0516)) * 60) / 0.10897435897;
    // // chassis rad/s to motor rpm
    // float psi_scale = ((.1835 / (PI * 2 * 0.0516)) * 60) / 0.10897435897;
    // // define coeff matracies for each system we want to solve
    // float coeff_matrix1[3][4] = { {x_scale,0,psi_scale,front_right},{0,-y_scale,psi_scale,back_right},{-x_scale,0,psi_scale,back_left} };
    // float coeff_matrix2[3][4] = { {x_scale,0,psi_scale,front_right},{0,-y_scale,psi_scale,back_right},{0,y_scale,psi_scale,front_left} };
    // float coeff_matrix3[3][4] = { {x_scale,0,psi_scale,front_right},{0,y_scale,psi_scale,front_left},{-x_scale,0,psi_scale,back_left} };
    // float coeff_matrix4[3][4] = { {0,-y_scale,psi_scale,back_right},{0,y_scale,psi_scale,front_left},{-x_scale,0,psi_scale,back_left} };

    // // 4 solution sets of x, y, psi
    // float vel_solutions[4][3];
    // solveSystem(coeff_matrix1, vel_solutions[0]);
    // solveSystem(coeff_matrix2, vel_solutions[1]);
    // solveSystem(coeff_matrix3, vel_solutions[2]);
    // solveSystem(coeff_matrix4, vel_solutions[3]);

    // float vel_estimate[3];

    // vel_estimate[0] = (cos(yaw_enc_angle - yaw_angle) * vel_solutions[0][0] + sin(yaw_enc_angle - yaw_angle) * vel_solutions[0][1]);
    // vel_estimate[1] = (-sin(yaw_enc_angle - yaw_angle) * vel_solutions[0][0] + cos(yaw_enc_angle - yaw_angle) * vel_solutions[0][1]);
    // vel_estimate[2] = vel_solutions[0][2];

    // integrate to find pos
    // pos_estimate[0] += vel_estimate[0] * dt;
    // pos_estimate[1] += vel_estimate[1] * dt;
    // pos_estimate[2] += vel_estimate[2] * dt;

    // output[0][1] = vel_estimate[0];
    output[0][2] = 0;
    // output[1][1] = vel_estimate[1];
    output[1][2] = 0;
    output[2][0] = chassis_angle; // chassis angle
    // output[2][1] = vel_estimate[2];
    output[2][2] = yaw_enc_angle;
}

FlyWheelEstimator::FlyWheelEstimator(CANManager* _can) {
    can = _can;
}

void FlyWheelEstimator::step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override) {
    //can
    float radius = 30 * 0.001; //meters
    float angular_velocity_l = -can->get_motor_state(CAN_2, 3).speed;
    float angular_velocity_r = can->get_motor_state(CAN_2, 4).speed;
    float angular_velocity_avg = (angular_velocity_l + angular_velocity_r) / 2;
    linear_velocity = angular_velocity_avg * radius; //m/s

    //ref
    projectile_speed_ref = ref->ref_data.launching_status.initial_speed;

    //weighted average
    output[0][1] = (projectile_speed_ref * ref_weight) + (linear_velocity * can_weight);
}

FeederEstimator::FeederEstimator(CANManager* _can) {
    can = _can;
}


void FeederEstimator::step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override) {
    //can
    float angular_velocity_motor = can->get_motor_state(CAN_2, 5).speed / (2 * PI) / 60;
    float angular_velocity_feeder = angular_velocity_motor / 36;
    balls_per_second_can = angular_velocity_feeder * 8;

    //ref
    balls_per_second_ref = ref->ref_data.launching_status.launching_frequency;

    output[0][1] = (balls_per_second_ref * ref_weight) + (balls_per_second_can * can_weight);

}


SwitcherEstimator::SwitcherEstimator(Config config, CANManager* _can, TOFSensor* tof) {
    can = _can;
    time_of_flight = tof;
    tof_sensor_offset = config.sensor_info[6][1];
    tof_scale = config.sensor_info[6][2];
}

void SwitcherEstimator::step_states(float output[STATE_LEN][3], float curr_state[STATE_LEN][3], int override) {
    dt = time.delta();
    //read tof sensor (millimeters)
    float tof_distance = ((float)(time_of_flight->read()) - tof_sensor_offset) / tof_scale;
    float angular_velocity_motor = -(((can->get_motor_state(CAN_2, 6).speed) / 36.0) * (5.1)) / tof_scale;
    total_motor_angle += (can->get_motor_state(CAN_2, 6).speed) * dt;
    // float rad_per_switch = 315;
    // if(total_motor_angle > rad_per_switch){
    //     total_motor_angle = rad_per_switch;
    // }
    // if(total_motor_angle < 0){
    //     total_motor_angle = 0;
    // }
    // float distance_from_right = -(total_motor_angle-(rad_per_switch/2.0))/(rad_per_switch/2.0);
    // if(count < 10){
    //     dt = 0;
    //     distance_from_right = tof_distance;
    //     count++;
    // }
    // distance_from_right = distance_from_right*0.99 + tof_distance*0.01;
    output[0][0] = tof_distance;
    output[0][1] = angular_velocity_motor;
}


LocalEstimator::LocalEstimator(CANManager* _can) {
    micro_estimator = true;
    can = _can;
}

void LocalEstimator::step_states(float output[CAN_MAX_MOTORS][MICRO_STATE_LEN], float curr_state[CAN_MAX_MOTORS][MICRO_STATE_LEN], int override) {
    for (size_t i = 0; i < CAN_NUM_BUSSES; i++) {
        for (size_t j = 0; j < CAN_MAX_MOTORS_PER_BUS; j++) {
            // j + 1 for motor ID on get_motor_state since j starts at 0 but it expects a 1-indexed ID
            if (can->get_motor(i, j + 1))
                output[(i * CAN_MAX_MOTORS_PER_BUS) + j][1] = can->get_motor_state(i, j + 1).speed;
        }
    }
}

