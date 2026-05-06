#include <cmath>
#include <limits>

#include "estimator.hpp"
#include "ICM20649.hpp"
#include "utils/vector_math.hpp"
#include "sensors/RefSystem.hpp"

// Estimator shared checking implementation
void Estimator::check_state_limits(const char* estimator_name, const char* state_name, const State& state, ErrorMonitor& monitor) {
    const Cfg::State& config = state.config();
    // Check position, velocity, acceleration against reference limits
    bool violated = false;
    float violation_amount = 0.0f;

    float pos = state.get_position();
    float vel = state.get_velocity();

    // Non-finite values indicate broken estimation and should fail safe immediately.
    if (!std::isfinite(pos) || !std::isfinite(vel)) {
        violated = true;
        violation_amount = std::numeric_limits<float>::quiet_NaN();
    } else if (pos < config.physical_limits.position.min) {
        violated = true;
        violation_amount = config.physical_limits.position.min - pos;
    } else if (pos > config.physical_limits.position.max) {
        violated = true;
        violation_amount = pos - config.physical_limits.position.max;
    }

    if (!violated) {
        if (vel < config.physical_limits.velocity.min) {
            violated = true;
            violation_amount = config.physical_limits.velocity.min - vel;
        } else if (vel > config.physical_limits.velocity.max) {
            violated = true;
            violation_amount = vel - config.physical_limits.velocity.max;
        }
    }

    // Acceleration reference limits are much less likely to be represent our robots true physical limits so ignore
    // float acc = state.get_acceleration();
    // if (!violated) {
    //     if (acc < config.physical_limits.acceleration.min) {
    //         violated = true;
    //         violation_amount = config.physical_limits.acceleration.min - acc;
    //     } else if (acc > config.physical_limits.acceleration.max) {
    //         violated = true;
    //         violation_amount = acc - config.physical_limits.acceleration.max;
    //     }
    // }

    if (violated) {
        if (!monitor.exceeding) {
            monitor.exceeding = true;
            monitor.exceed_start_us = micros();
        }
        const uint32_t elapsed_us = static_cast<uint32_t>(micros() - monitor.exceed_start_us);
        // If state violation exceeds the configured threshold for too long, trigger the error handler. Setting max_error_exceed_time_us to 0 means immediate escalation.
        if (elapsed_us >= config.max_error_exceed_time_us) {
            handleEstimatorError(estimator_name, state_name, state, violation_amount);
        }
    } else {
        monitor.exceeding = false;
        monitor.exceed_start_us = 0;
    }
}

void Estimator::handleEstimatorError(const char* estimator_name, const char* state_name, const State& state, float violation_amount) {
    const Cfg::State& config = state.config();
    safety::safety_procedure(
        "%s: %s estimate exceeded reference limits by %f (STATE: pos: %f, vel: %f, acc: %f; LIMITS: pos:[%f,%f] vel:[%f,%f] acc:[%f,%f])",
        estimator_name,
        state_name,
        violation_amount,
        state.get_position(),
        state.get_velocity(),
        state.get_acceleration(),
        config.reference_limits.position.min,
        config.reference_limits.position.max,
        config.reference_limits.velocity.min,
        config.reference_limits.velocity.max,
        config.reference_limits.acceleration.min,
        config.reference_limits.acceleration.max
    );
}

void GimbalAndChassisEstimator::validate(RobotStateMap& updated_state_map) {
    check_state_limits("GimbalAndChassisEstimator", "Chassis X", updated_state_map[chassis_x_state], chassis_x_monitor);
    check_state_limits("GimbalAndChassisEstimator", "Chassis Y", updated_state_map[chassis_y_state], chassis_y_monitor);
    check_state_limits("GimbalAndChassisEstimator", "Chassis Heading", updated_state_map[chassis_heading_state], chassis_heading_monitor);
    check_state_limits("GimbalAndChassisEstimator", "Yaw", updated_state_map[yaw_state], yaw_monitor);
    check_state_limits("GimbalAndChassisEstimator", "Pitch", updated_state_map[pitch_state], pitch_monitor);
}

GimbalAndChassisEstimator::GimbalAndChassisEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states) : 
    buff_enc_yaw(sensor_manager.get_sensor_by_name<BuffEncoder>(estimator_config.get_sensor_name_by_generic_use(Cfg::GenericSensorUse::YawBuffEncoder))), 
    buff_enc_pitch(sensor_manager.get_sensor_by_name<BuffEncoder>(estimator_config.get_sensor_name_by_generic_use(Cfg::GenericSensorUse::PitchBuffEncoder))),
    icm_imu(sensor_manager.get_sensor_by_name<ICM20649>(estimator_config.get_sensor_name_by_generic_use(Cfg::GenericSensorUse::YawIcmImu))),
    chassis_x_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::ChassisX, estimator_config, available_states)),
    chassis_y_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::ChassisY, estimator_config, available_states)),
    chassis_heading_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::ChassisHeading, estimator_config, available_states)),
    yaw_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::GimbalYaw, estimator_config, available_states)),
    pitch_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::GimbalPitch, estimator_config, available_states)) {

    printf("State names: chassis x: %lu, chassis y: %lu, chassis heading: %lu, yaw: %lu, pitch: %lu\n", 
        static_cast<uint32_t>(chassis_x_state), static_cast<uint32_t>(chassis_y_state), static_cast<uint32_t>(chassis_heading_state), 
        static_cast<uint32_t>(yaw_state), static_cast<uint32_t>(pitch_state));
    
    Serial.println("Initializing Gimbal and Chassis Estimator");
    chassis_1 = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::ChassisFrontRight));
    chassis_2 = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::ChassisBackRight));
    chassis_3 = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::ChassisBackLeft));
    chassis_4 = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::ChassisFrontLeft));
    Serial.println("Gimbal and Chassis Estimator initialized!");
    yaw_encoder_offset = estimator_config.sensor_info.yaw_encoder_offset;
    pitch_encoder_offset = estimator_config.sensor_info.pitch_encoder_offset;
    
    pitch_encoder_direction = estimator_config.sensor_info.pitch_encoder_direction;
    yaw_encoder_direction = estimator_config.sensor_info.yaw_encoder_direction;

    has_pitch_imu = static_cast<bool>(estimator_config.sensor_info.has_pitch_imu);

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
    float pitch_enc_angle = (buff_enc_pitch->get_angle() * pitch_encoder_direction) - pitch_encoder_offset;
    while (pitch_enc_angle >= PI)
        pitch_enc_angle -= 2 * PI;
    while (pitch_enc_angle <= -PI)
        pitch_enc_angle += 2 * PI;

    float yaw_enc_angle = (buff_enc_yaw->get_angle() * yaw_encoder_direction) - yaw_encoder_offset;
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

    if (has_pitch_imu) {
        yaw_axis_spherical[2] = acos(imu_yaw_axis_vector[2] / Utils::magnitude(imu_yaw_axis_vector, 3)) - pitch_diff; // phi
    } else {
        yaw_axis_spherical[2] = acos(imu_yaw_axis_vector[2] / Utils::magnitude(imu_yaw_axis_vector, 3)); // phi
    }

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
    if (dt > .1) {
        dt = 0; // first dt loop generates huge time so check for that
        current_pitch_velocity = 0;
    } else {
        current_pitch_velocity = (pitch_enc_angle - previous_state_map[pitch_state].get_position()) / dt;
    }

    yaw_angle += current_yaw_velocity * (dt);
    pitch_angle += current_pitch_velocity * (dt);
    roll_angle += current_roll_velocity * (dt);

    global_yaw_angle += -global_yaw_velocity * (dt);
    global_pitch_angle += -global_pitch_velocity * (dt);
    global_roll_angle += -global_roll_velocity * (dt);

    // chassis_angle = yaw_angle - yaw_enc_angle;
    chassis_angle = yaw_enc_angle;
    if (count1 == 0) {
        initial_chassis_angle = chassis_angle;
        prev_global_chassis_angle = chassis_angle;
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
    
    updated_state_map[yaw_state].set_position_no_bound(yaw_angle);
    updated_state_map[yaw_state].set_velocity_no_bound(current_yaw_velocity);
    updated_state_map[yaw_state].set_acceleration_no_bound(roll_angle);
    updated_state_map[pitch_state].set_position_no_bound(pitch_enc_angle);
    updated_state_map[pitch_state].set_velocity_no_bound(current_pitch_velocity);
    updated_state_map[pitch_state].set_acceleration_no_bound(0);
    updated_state_map[chassis_heading_state].set_position_no_bound(chassis_angle);

    

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
    float global_chassis_angle = yaw_angle - yaw_enc_angle;
    while (global_chassis_angle >= PI)
        global_chassis_angle -= 2 * PI;
    while (global_chassis_angle <= -PI)
        global_chassis_angle += 2 * PI;
    // global_chassis_angle = -(total_odom_pos[0] + total_odom_pos[2])/(2*odom_axis_offset_x)+initial_global_chassis_angle;  
    float d_chassis_heading = (global_chassis_angle - prev_global_chassis_angle);
    if (d_chassis_heading > PI) d_chassis_heading -= 2 * PI;
    else if (d_chassis_heading < -PI) d_chassis_heading += 2 * PI;
    prev_global_chassis_angle = global_chassis_angle;
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


    updated_state_map[chassis_x_state].set_position_no_bound(pos_estimate[0]);
    // output[0][1] = (pos_estimate[0] - previous_pos[0]) / dt;
    updated_state_map[chassis_x_state].set_velocity_no_bound(vel_estimate[0]);
    updated_state_map[chassis_x_state].set_acceleration_no_bound(0);

    updated_state_map[chassis_y_state].set_position_no_bound(pos_estimate[1]);
    // output[1][1] = (pos_estimate[1] - previous_pos[1]) / dt;
    updated_state_map[chassis_y_state].set_velocity_no_bound(vel_estimate[1]);
    updated_state_map[chassis_y_state].set_acceleration_no_bound(0);

    updated_state_map[chassis_heading_state].set_velocity_no_bound(d_chassis_heading / dt);
    updated_state_map[chassis_heading_state].set_acceleration_no_bound(0);


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
    updated_state_map[ball_exit_velocity].set_velocity_no_bound((projectile_speed_ref * ref_estimate_weight) + (linear_velocity * motor_estimate_weight));
}

void FlywheelEstimator::validate(RobotStateMap& updated_state_map) {
    check_state_limits("FlywheelEstimator", "Flywheel Velocity", updated_state_map[ball_exit_velocity], flywheel_monitor);
}

FeederEstimator::FeederEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states) :
    feeder_ball_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::FeederBallPosition, estimator_config, available_states)),
    feeder_encoder(sensor_manager.get_sensor_by_name<BuffEncoder>(estimator_config.get_sensor_name_by_generic_use(Cfg::GenericSensorUse::FeederBuffEncoder))) {
    feeder_offset = estimator_config.sensor_info.feeder_encoder_offset;
    feeder_direction = estimator_config.sensor_info.feeder_direction;
    feeder_ratio = estimator_config.sensor_info.feeder_ratio;
}

void FeederEstimator::step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override) {
    dt = time.delta();
    float feeder_angle = feeder_encoder->get_angle();
    float diff;
    if (count == 0) {
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
    updated_state_map[feeder_ball_state].set_position_no_bound(ball_count * feeder_direction); // ball count
    updated_state_map[feeder_ball_state].set_velocity_no_bound(feeder_velocity * feeder_direction); // ball velocity
    updated_state_map[feeder_ball_state].set_acceleration_no_bound(0); // this is not the acceleration just the encoder value for debugging

}

void FeederEstimator::validate(RobotStateMap& updated_state_map) {
    check_state_limits("FeederEstimator", "Feeder", updated_state_map[feeder_ball_state], feeder_monitor);
}

void LowerFeederEstimator::validate(RobotStateMap& updated_state_map) {
    check_state_limits("LowerFeederEstimator", "Lower Feeder", updated_state_map[feeder_ball_state], lower_feeder_monitor);
}

LowerFeederEstimator::LowerFeederEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states) :
    feeder_ball_state(get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse::LowerFeederBallPosition, estimator_config, available_states)),
    feeder_encoder(sensor_manager.get_sensor_by_name<BuffEncoder>(estimator_config.get_sensor_name_by_generic_use(Cfg::GenericSensorUse::FeederBuffEncoder))) {
    feeder_offset = estimator_config.sensor_info.feeder_encoder_offset;
    feeder_direction = estimator_config.sensor_info.feeder_direction;
    feeder_ratio = estimator_config.sensor_info.feeder_ratio;

    near_feeder_motor = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::FeederClose));
    far_feeder_motor = can.get_motor_by_name(estimator_config.get_motor_name_by_generic_use(Cfg::GenericEstimatorMotorUse::FeederFar));
}

void LowerFeederEstimator::step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override) {
    dt = time.delta();
    float feeder_angle = feeder_encoder->get_angle();
    float diff;
    if (count == 0) {
        dt = 0; // first dt loop generates huge time so check for that
        diff = fmod((feeder_angle - feeder_offset), (float)(M_PI / feeder_ratio)) ;
        count++;
    } else {
        diff = feeder_angle - prev_feeder_angle;
    }

    // code to check if the encoder value is getting reset to near 0. Indicates a problem with the spi bus
    if (fabs(diff) > 1 && fabs(diff) < 5) {
        num_encoder_resets++;
        reset_value = feeder_angle;
        Serial.printf("Feeder angle diff is large: %d, feeder angle: %f, prev feeder angle: %f\n", num_encoder_resets, feeder_angle, prev_feeder_angle);
    }

    prev_feeder_angle = feeder_angle;
    if (diff > PI) diff -= 2 * PI;
    else if (diff < -PI) diff += 2 * PI;

    // velocity estimation with buff encoder
    float feeder_velocity = (dt > 0) ? (diff/(M_PI/feeder_ratio))/dt : 0;
  
    ball_count += diff/(M_PI/feeder_ratio);

    updated_state_map[feeder_ball_state].set_position_no_bound(ball_count * feeder_direction); // ball count
    updated_state_map[feeder_ball_state].set_velocity_no_bound(feeder_velocity * feeder_direction); // ball velocity
    updated_state_map[feeder_ball_state].set_acceleration_no_bound(0); // this is not the acceleration just the encoder value for debugging
}

