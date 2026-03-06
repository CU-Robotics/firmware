#include "estimator.hpp"
#include "ICM20649.hpp"
#include "utils/vector_math.hpp"
#include "sensors/RefSystem.hpp"

GimbalAndChassisEstimator::GimbalAndChassisEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states) : 
    buff_enc_yaw(sensor_manager.get_sensor_by_name<BuffEncoder>(estimator_config.get_sensor_name_by_generic_use(Cfg::GenericSensorUse::YawBuffEncoder))), 
    buff_enc_pitch(sensor_manager.get_sensor_by_name<BuffEncoder>(estimator_config.get_sensor_name_by_generic_use(Cfg::GenericSensorUse::PitchBuffEncoder))),
    icm_imu(sensor_manager.get_sensor_by_name<ICM20649>(estimator_config.get_sensor_name_by_generic_use(Cfg::GenericSensorUse::YawIcmImu))),
    chassis_x_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::ChassisX, estimator_config, available_states)),
    chassis_y_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::ChassisY, estimator_config, available_states)),
    chassis_heading_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::ChassisHeading, estimator_config, available_states)),
    yaw_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::GimbalYaw, estimator_config, available_states)),
    pitch_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::GimbalPitch, estimator_config, available_states)) {

    chassis_1 = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::ChassisFrontRight));
    chassis_2 = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::ChassisBackRight));
    chassis_3 = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::ChassisBackLeft));
    chassis_4 = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::ChassisFrontLeft));
    
    yaw_encoder_offset = estimator_config.sensor_info.yaw_encoder_offset;
    pitch_encoder_offset = estimator_config.sensor_info.pitch_encoder_offset;

    yaw_angle = estimator_config.sensor_info.yaw_start_angle;
    pitch_angle = estimator_config.sensor_info.pitch_start_angle;
    roll_angle = estimator_config.sensor_info.roll_start_angle;
    chassis_angle = 0;
    imu_yaw_axis_vector[0] = estimator_config.sensor_info.yaw_axis_vector[0];
    imu_yaw_axis_vector[1] = estimator_config.sensor_info.yaw_axis_vector[1];
    imu_yaw_axis_vector[2] = estimator_config.sensor_info.yaw_axis_vector[2];
    imu_pitch_axis_vector[0] = estimator_config.sensor_info.pitch_axis_vector[0];
    imu_pitch_axis_vector[1] = estimator_config.sensor_info.pitch_axis_vector[1];
    imu_pitch_axis_vector[2] = estimator_config.sensor_info.pitch_axis_vector[2];
    starting_pitch_angle = estimator_config.sensor_info.pitch_angle_at_imu_calibration;

    chassis_x_to_motor_rad = estimator_config.sensor_info.chassis_x_to_motor_rad;
    chassis_y_to_motor_rad = estimator_config.sensor_info.chassis_y_to_motor_rad;
    chassis_rad_to_motor_rad = estimator_config.sensor_info.chassis_rad_to_motor_rad;

    // odom_wheel_radius = estimator_config.sensor_info.odom_wheel_radius;
    // odom_axis_offset_x = estimator_config.sensor_info.odom_axis_offset_x;
    // odom_axis_offset_y = estimator_config.sensor_info.odom_axis_offset_y;
    // definitions for spherical coordinates of new axis in the imu refrence frame
    yaw_axis_spherical[0] = 1;   // rho (1 for a spherical)
    pitch_axis_spherical[0] = 1; // rho (1 for a spherical)
    roll_axis_spherical[0] = 1;  // rho (1 for a spherical)
}

void GimbalAndChassisEstimator::step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override) {
    float pitch_enc_angle = (-buff_enc_pitch->get_angle()) - pitch_encoder_offset;
    while (pitch_enc_angle >= PI)
        pitch_enc_angle -= 2 * PI;
    while (pitch_enc_angle <= -PI)
        pitch_enc_angle += 2 * PI;

    float yaw_enc_angle = (buff_enc_yaw->get_angle()) - yaw_encoder_offset;
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
    yaw_axis_spherical[2] = acos(imu_yaw_axis_vector[2] / Utils::magnitude(imu_yaw_axis_vector, 3)) - pitch_diff; // phi

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

    Utils::crossProduct(pitch_axis_unitvector, yaw_axis_unitvector, roll_axis_unitvector);

    float magicNum = 0; // left yaw increases with 0.8
    Utils::rotateVector3D(roll_axis_unitvector, yaw_axis_unitvector, magicNum, yaw_axis_unitvector);
    Utils::rotateVector3D(roll_axis_unitvector, pitch_axis_unitvector, magicNum, pitch_axis_unitvector);

    // rotateVector3D(yaw_axis_unitvector,roll_axis_unitvector,(PI*0.5),pitch_axis_unitvector);

    // offset the axis' based on the pitch yaw and roll data, These vectors give global pitch yaw and roll
    // rotateVector3D(roll_axis_unitvector, yaw_axis_unitvector, global_roll_angle, yaw_axis_global);
    // rotateVector3D(roll_axis_unitvector, pitch_axis_unitvector, global_roll_angle, pitch_axis_global);

    // rotateVector3D(pitch_axis_unitvector, yaw_axis_unitvector, (global_pitch_angle - pitch_enc_angle), yaw_axis_global);
    // rotateVector3D(pitch_axis_unitvector, roll_axis_unitvector, (global_pitch_angle - pitch_enc_angle), roll_axis_global);

    // gets the velocity data from the imu and uses the gravity vector to calculate the yaw velocity
    float raw_omega_vector[3] = { icm_imu->get_gyro_X(), icm_imu->get_gyro_Y(), icm_imu->get_gyro_Z() };

    // *Note: X is pitch Y is Roll Z is Yaw, when level
    // positive pitch angle is up, positive roll angle is right(robot pov), positive yaw is left(robot pov)

    float temp1[3];
    float temp2[3];
    float temp3[3];
    Utils::crossProduct(yaw_axis_unitvector, pitch_axis_unitvector, temp1);
    Utils::crossProduct(yaw_axis_unitvector, roll_axis_unitvector, temp2);
    Utils::crossProduct(roll_axis_unitvector, pitch_axis_unitvector, temp3);

    // update previous to the current value before current is updated
    previous_pitch_velocity = current_pitch_velocity;
    previous_yaw_velocity = current_yaw_velocity;
    previous_roll_velocity = current_roll_velocity;

    float imu_vel_offset = 1;
    // calculate the pitch yaw and roll velocities (Gimbal Relative)
    current_pitch_velocity = Utils::vectorProduct(pitch_axis_unitvector, raw_omega_vector, 3) / imu_vel_offset;
    current_yaw_velocity = Utils::vectorProduct(yaw_axis_unitvector, raw_omega_vector, 3) / imu_vel_offset;
    current_roll_velocity = -Utils::vectorProduct(roll_axis_unitvector, raw_omega_vector, 3) / imu_vel_offset;

    // calculate the pitch yaw and roll velocities (Global Reference)
    global_pitch_velocity = Utils::vectorProduct(pitch_axis_global, raw_omega_vector, 3);
    global_yaw_velocity = Utils::vectorProduct(yaw_axis_global, raw_omega_vector, 3);
    global_roll_velocity = Utils::vectorProduct(roll_axis_global, raw_omega_vector, 3);
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
    updated_state_map[yaw_state].set_position(yaw_angle);
    updated_state_map[yaw_state].set_velocity(current_yaw_velocity);
    updated_state_map[yaw_state].set_acceleration(roll_angle);
    updated_state_map[pitch_state].set_position(pitch_enc_angle);
    updated_state_map[pitch_state].set_velocity(current_pitch_velocity);
    updated_state_map[pitch_state].set_acceleration(pitch_enc_angle);

    // 3 odom wheel estimation
    // for (int i = 0; i < 3; i++) {
    //     curr_rev_raw[i] = rev_enc[i]->get_angle_radians();

    //     if ((curr_rev_raw[i] - prev_rev_raw[i]) > PI) rev_diff[i] = ((curr_rev_raw[i] - prev_rev_raw[i]) - (2 * PI));
    //     else if ((curr_rev_raw[i] - prev_rev_raw[i]) < -PI) rev_diff[i] = ((curr_rev_raw[i] - prev_rev_raw[i]) + (2 * PI));
    //     else rev_diff[i] = (curr_rev_raw[i] - prev_rev_raw[i]);
    //     prev_rev_raw[i] = curr_rev_raw[i];
    //     odom_pos_diff[i] = rev_diff[i] * odom_wheel_radius;
    //     total_odom_pos[i] = odom_pos_diff[i] + total_odom_pos[i];
    // }

    chassis_angle = yaw_angle - yaw_enc_angle;
    while (chassis_angle >= PI)
        chassis_angle -= 2 * PI;
    while (chassis_angle <= -PI)
        chassis_angle += 2 * PI;
    // chassis_angle = -(total_odom_pos[0] + total_odom_pos[2])/(2*odom_axis_offset_x)+initial_chassis_angle;  
    float d_chassis_heading = (chassis_angle - prev_chassis_angle);
    if (d_chassis_heading > PI) d_chassis_heading -= 2 * PI;
    else if (d_chassis_heading < -PI) d_chassis_heading += 2 * PI;
    prev_chassis_angle = chassis_angle;
    if (override == 1) {
        pos_estimate[0] = previous_state_map[chassis_x_state].get_position();
        pos_estimate[1] = previous_state_map[chassis_y_state].get_position();
        previous_pos[0] = previous_state_map[chassis_x_state].get_position();
        previous_pos[1] = previous_state_map[chassis_y_state].get_position();
    }
    // chassis estimation
    float front = chassis_1->get_state().speed;
    float right = chassis_2->get_state().speed;
    float back = chassis_3->get_state().speed;
    float left = chassis_4->get_state().speed;
    
    // m/s of chassis to motor rad/s
    float x_scale = chassis_x_to_motor_rad;
    float y_scale = chassis_y_to_motor_rad;
    // chassis rad/s to motor rad/s
    float psi_scale = chassis_rad_to_motor_rad;
    // define coeff matracies for each system we want to solve
    float coeff_matrix1[3][4] = { {0,y_scale,psi_scale,front},{x_scale,0,psi_scale,right},{0,-y_scale,psi_scale,back} };
    float coeff_matrix2[3][4] = { {0,y_scale,psi_scale,front},{x_scale,0,psi_scale,right},{-x_scale,0,psi_scale,left} };
    float coeff_matrix3[3][4] = { {0,y_scale,psi_scale,front},{-x_scale,0,psi_scale,left},{0,-y_scale,psi_scale,back} };
    float coeff_matrix4[3][4] = { {x_scale,0,psi_scale,right},{-x_scale,0,psi_scale,left},{0,-y_scale,psi_scale,back} };

    // 4 solution sets of x, y, psi
    float vel_solutions[4][3];
    Utils::solveSystem(coeff_matrix1, vel_solutions[0]);
    Utils::solveSystem(coeff_matrix2, vel_solutions[1]);
    Utils::solveSystem(coeff_matrix3, vel_solutions[2]);
    Utils::solveSystem(coeff_matrix4, vel_solutions[3]);

    float vel_estimate[3];

    vel_estimate[0] = (cos(yaw_enc_angle - yaw_angle) * vel_solutions[0][0] + sin(yaw_enc_angle - yaw_angle) * vel_solutions[0][1]);
    vel_estimate[1] = (-sin(yaw_enc_angle - yaw_angle) * vel_solutions[0][0] + cos(yaw_enc_angle - yaw_angle) * vel_solutions[0][1]);
    vel_estimate[2] = vel_solutions[0][2];

    // integrate to find pos
    pos_estimate[0] += vel_estimate[0] * dt;
    pos_estimate[1] += vel_estimate[1] * dt;
    pos_estimate[2] += vel_estimate[2] * dt;



    updated_state_map[chassis_x_state].set_position(pos_estimate[0]);
    // output[0][1] = (pos_estimate[0] - previous_pos[0]) / dt;
    updated_state_map[chassis_x_state].set_velocity(vel_estimate[0]);
    updated_state_map[chassis_x_state].set_acceleration(0);

    updated_state_map[chassis_y_state].set_position(pos_estimate[1]);
    // output[1][1] = (pos_estimate[1] - previous_pos[1]) / dt;
    updated_state_map[chassis_y_state].set_velocity(vel_estimate[1]);
    updated_state_map[chassis_y_state].set_acceleration(0);

    updated_state_map[chassis_heading_state].set_position(chassis_angle);
    updated_state_map[chassis_heading_state].set_velocity(d_chassis_heading / dt);
    updated_state_map[chassis_heading_state].set_acceleration(yaw_enc_angle);



    previous_pos[0] = pos_estimate[0];
    previous_pos[1] = pos_estimate[1];
}

FlywheelEstimator::FlywheelEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states) : 
ball_exit_velocity(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::ShooterBallVelocity, estimator_config, available_states)) {
    flywheel_motor_left = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::FlywheelLeft));
    flywheel_motor_right = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::FlywheelRight));

    flywheel_radius = estimator_config.sensor_info.flywheel_radius;

    motor_estimate_weight = estimator_config.sensor_info.flywheel_motor_estimate_weight;
    ref_estimate_weight = 1 - motor_estimate_weight;
}

void FlywheelEstimator::step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override) {
    float angular_velocity_l = -flywheel_motor_left->get_state().speed; //motor speed is in rad/s and negative because of orientation
    float angular_velocity_r = flywheel_motor_right->get_state().speed; //motor speed is in rad/s
    float angular_velocity_avg = (angular_velocity_l + angular_velocity_r) / 2;
    linear_velocity = angular_velocity_avg * flywheel_radius; //m/s

    // ball speed measured by the speed monitor module
    projectile_speed_ref = ref.ref_data.launching_status.initial_speed;

    //weighted average
    updated_state_map[ball_exit_velocity].set_velocity((projectile_speed_ref * ref_estimate_weight) + (linear_velocity * motor_estimate_weight));
}

NewFeederEstimator::NewFeederEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states) :
    feeder_ball_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::FeederBallPosition, estimator_config, available_states)),
    feeder_encoder(sensor_manager.get_sensor_by_name<BuffEncoder>(estimator_config.get_sensor_name_by_generic_use(Cfg::GenericSensorUse::FeederBuffEncoder))) {
    feeder_offset = estimator_config.sensor_info.feeder_encoder_offset;
    feeder_direction = estimator_config.sensor_info.feeder_direction;
    feeder_ratio = estimator_config.sensor_info.feeder_ratio;
}

void NewFeederEstimator::step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override) {
    dt = time.delta();
    float feeder_angle = feeder_encoder->get_angle();
    // Serial.printf("waggle graph feeder_angle %f\n",feeder_angle);
    // Serial.printf("feeder_offset %f\n",feeder_offset);
    float diff;
    if (count == 0) {
        Serial.printf("prev_feeder_angle %f\n",prev_feeder_angle);
        dt = 0; // first dt loop generates huge time so check for that
        diff = fmod((feeder_angle - feeder_offset), (float)(M_PI / feeder_ratio)) ;
        count++;
    } else {
        diff = feeder_angle - prev_feeder_angle;
    }
  
    prev_feeder_angle = feeder_angle;
    if (diff > PI) diff -= 2 * PI;
    else if (diff < -PI) diff += 2 * PI;
  
    float feeder_velocity = (dt > 0) ? (diff/(M_PI/feeder_ratio))/dt : 0;
  
    ball_count += diff/(M_PI/feeder_ratio);
    updated_state_map[feeder_ball_state].set_position(ball_count * feeder_direction); // ball count
    updated_state_map[feeder_ball_state].set_velocity(feeder_velocity * feeder_direction); // ball velocity
    updated_state_map[feeder_ball_state].set_acceleration(feeder_angle); // this is not the acceleration just the encoder value for debugging

}

