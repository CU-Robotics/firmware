#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include "estimator.hpp"
#include "robot_state_map.hpp"
#include "safety.hpp"
#include "sensors/can/can_manager.hpp"
#include "state.hpp"
#include "comms/config_layer.hpp"

#include "sensors/buff_encoder.hpp"
#include "sensors/rev_encoder.hpp"
#include "sensors/ICM20649.hpp"
#include "sensors/sensor_manager.hpp"


/// @brief Parent estimator struct. All estimators should inherit from this.
struct Estimator {
public:
    Estimator() { };

    virtual ~Estimator() { };

    /// @brief step the current state(s) and update the estimate array accordingly
    /// @param outputs estimated state array to update with certain estimated states
    /// @param curr_state current state array to update with new state
    /// @param override true if we want to override the current state with the new state
    virtual void step_states(RobotStateMap updated_state_map, RobotStateMap previous_state_map, int override) = 0;
    
    const Cfg::StateName& get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse use, Cfg::Estimator estimator_config, std::vector<Cfg::StateName> available_states) {
        const Cfg::StateName& requested_state_name = estimator_config.get_state_name_by_generic_use(use);
        for (const auto& state_name : available_states) {
            if (state_name == requested_state_name) {
                return requested_state_name;
            }
        }
        safety::safety_procedure("Estimator requesting state that is not available. (multiple estimators may be trying to estimate the same state)");
    }

protected:
    ///@brief create a timer object for each estimator
    Timer time;
};

/// @brief Estimate the yaw, pitch, and chassis heading
struct GimbalAndChassisEstimator : public Estimator {
private:
    /// @brief angle offset because calibration does not allign to our coordiate system
    float yaw_encoder_offset;
    /// @brief angle offset because calibration does not allign to our coordiate system
    float pitch_encoder_offset;
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

    float chassis_x_to_motor_rad;
    float chassis_y_to_motor_rad;
    float chassis_rad_to_motor_rad;

    std::shared_pointer<BuffEncoder> buff_enc_yaw;
    std::shared_pointer<BuffEncoder> buff_enc_pitch;
    std::shared_pointer<ICM20649> icm_imu;

    Motor* chassis_1;
    Motor* chassis_2;
    Motor* chassis_3;
    Motor* chassis_4;

    const Cfg::StateName& chassis_x_state;
    const Cfg::StateName& chassis_y_state;
    const Cfg::StateName& chassis_heading_state;
    const Cfg::StateName& yaw_state;
    const Cfg::StateName& pitch_state;

    /// @brief position estimate to store position after integrating used for chassis odometry
    float pos_estimate[3] = { 0,0,0 };

    /// @brief previous pose to store the previous pose for chassis odometry
    float previous_pos[3] = { 0,0,0 };

public:
    /// @brief estimate the state of the gimbal
    /// @param config_data inputted sensor values from Hive yaml
    /// @param sensor_manager sensor manager object 
    /// @param can can from Estimator Manager
    GimbalAndChassisEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states);

    /// @brief calculate estimated states and add to output array
    /// @param output output array to add estimated states to
    /// @param curr_state current state of the system
    /// @param override override the current state
    void step_states(RobotStateMap& updated_state_map, RobotStateMap& previous_state_map, int override) override;
};

/// @brief Estimate the state of the flywheels as meters/second of balls exiting the barrel.
struct FlywheelEstimator : public Estimator {
private:
    /// @brief can pointer from EstimatorManager
    CANManager* can;
    /// @brief calculated flywheel state from ref
    float projectile_speed_ref;

    /// @brief calculated flywheel state from can
    float linear_velocity;
    /// @brief can weight for weighted average
    float motor_estimate_weight = 1;

    /// @brief ref weight for weighted average
    float ref_estimate_weight = 0;

    Motor* flywheel_motor_left;
    Motor* flywheel_motor_right;

    const Cfg::StateName& ball_exit_velocity;

public:
    /// @brief make new flywheel estimator and set can data pointer and num states
    /// @param can can pointer from EstimatorManager
    FlywheelEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states);

    /// @brief generate estimated states and replace in output array
    /// @param output array to be updated with the calculated states
    /// @param curr_state current state of the flywheel
    /// @param override override flag
    void step_states(RobotStateMap& updated_state_map, RobotStateMap& previous_state_map, int override);
};

/// @brief This estimator estimates our "micro" state which is stores all the motor velocities(in rad/s), whereas the other estimators estimate "macro" state which stores robot joints
struct NewFeederEstimator : public Estimator {
    private:
        /// @brief can from EstimatorManager
        CANManager* can;
        /// @brief sensor manager
        SensorManager* sensor_manager;
        /// @brief delta time
        float dt = 0;
        /// @brief previous feeder angle
        float prev_feeder_angle = 0;
        /// @brief previous ball count
        float ball_count = 0;
        /// @brief first loop counter
        int count = 0;
        /// @brief feeder encoder offset for improving feeder delay
        float feeder_offset = 0;
        /// @brief feeder direction multiplier
        /// @note 1 for normal direction, -1 for reverse direction
        float feeder_direction = 1;
        /// @brief gear ratio of the feeder
        float feeder_ratio = 1;

        const Cfg::StateName& feeder_ball_state;

        std::shared_pointer<BuffEncoder> feeder_encoder;
    public:
        /// @brief Make new local estimator and set can data pointer and num states
        /// @param can can pointer from EstimatorManager
        /// @param sensor_manager sensor manager object
        /// @param config_data config data from yaml
        NewFeederEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states);
    
        /// @brief step through each motor and add to micro state
        /// @param output entire micro state 
        /// @param curr_state current micro state
        /// @param override override flag
        void step_states(RobotStateMap& updated_state_map, RobotStateMap& previous_state_map, int override) override;
    };

#endif
