#pragma once

#include "estimator.hpp"
#include "robot_state_map.hpp"
#include "safety.hpp"
#include "sensors/can/can_manager.hpp"
#include "state.hpp"
#include "utils/timing.hpp"

#include "sensors/buff_encoder.hpp"
#include "sensors/rev_encoder.hpp"
#include "sensors/ICM20649.hpp"
#include "sensors/sensor_manager.hpp"
#include <memory>


/// @brief Parent estimator struct. All estimators should inherit from this.
struct Estimator {
public:
    /// @brief default constructor
    Estimator() { };
    /// @brief default destructor
    virtual ~Estimator() { };

    /// @brief step the current state(s) and update the estimate array accordingly
    /// @param updated_state_map the map of states to update with the new estimates
    /// @param previous_state_map the map of states with the previous estimates.
    /// @param override whether the current estimate is being overriden by an incoming override state from hive.
    virtual void step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override) = 0;

    /// @brief Validate estimator outputs after stepping.
    /// Managers call this so limit checks live outside the estimator step logic.
    /// @param updated_state_map the current estimate map produced by the estimators
    virtual void validate(const RobotStateMap& updated_state_map) { }
    /// @brief Helper function to get a state name by its generic use. Will trigger safety procedure if the state is not available.
    /// @param use the generic use of the state to get
    /// @param estimator_config config data for this estimator to get the requested state name from
    /// @param available_states vector of state names that are available to be estimated; this is to prevent multiple estimators from trying to estimate the same state
    /// @return the state name that corresponds to the generic use requested
    const Cfg::StateName& get_state_name_by_generic_use(Cfg::GenericEstimatorStateUse use, const Cfg::Estimator& estimator_config, std::vector<Cfg::StateName> available_states) {
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

    /// @brief Internal tracking for estimator exceedance on a single state.
    struct ErrorMonitor {
        /// @brief Whether the error monitor has been initialized
        bool initialized = false;
        /// @brief Whether the state estimate is currently exceeding physical limits
        bool exceeding = false;
        /// @brief Timestamp in microseconds when the limit exceedance first occurred
        uint32_t exceed_start_us = 0;
    };

    /// @brief Check whether a state estimate exceeds its configured reference limits and dispatch to the estimator-specific handler.
    /// @param estimator_name Name of the estimator for diagnostics.
    /// @param state_name Name of the state being checked.
    /// @param state The state object to check.
    /// @param monitor Persistent monitor for this state.
    void check_state_limits(const char* estimator_name, const char* state_name, const State& state, ErrorMonitor& monitor);

    /// @brief Handle an estimator-specific limit violation once the exceedance duration has been reached. Default calls safety procedure; override to customize.
    /// @param estimator_name Name of the estimator for diagnostics.
    /// @param state_name Name of the state whose estimate violated limits.
    /// @param state The state object that exceeded its physical limits.
    /// @param violation_amount How far the estimate exceeded its limit.
    virtual void handleEstimatorError(const char* estimator_name, const char* state_name, const State& state, float violation_amount);
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

    /// @brief the direction of the pitch encoder
    float pitch_encoder_direction;
    /// @brief the direction of the yaw encoder
    float yaw_encoder_direction;
    /// @brief whether or not the IMU is mounted on the pitch axis
    bool has_pitch_imu;

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
    float prev_global_chassis_angle = 0;
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

    /// @brief ratio to convert from chassis x meters to chassis motor radians
    float chassis_x_to_motor_rad;
    /// @brief ratio to convert from chassis y meters to chassis motor radians
    float chassis_y_to_motor_rad;
    /// @brief ratio to convert from chassis heading radians to chassis motor radians
    float chassis_rad_to_motor_rad;

    /// @brief BuffEncoder on the yaw
    std::shared_ptr<BuffEncoder> buff_enc_yaw;
    /// @brief BuffEncoder on the pitch
    std::shared_ptr<BuffEncoder> buff_enc_pitch;
    /// @brief ICM20649 IMU
    std::shared_ptr<ICM20649> icm_imu;

    /// @brief chassis motor 1
    std::shared_ptr<Motor> chassis_1;
    /// @brief chassis motor 2
    std::shared_ptr<Motor> chassis_2;
    /// @brief chassis motor 3
    std::shared_ptr<Motor> chassis_3;
    /// @brief chassis motor 4
    std::shared_ptr<Motor> chassis_4;

    /// @brief state name for the chassis x axis
    const Cfg::StateName& chassis_x_state;
    /// @brief state name for the chassis y axis
    const Cfg::StateName& chassis_y_state;
    /// @brief state name for the chassis heading axis
    const Cfg::StateName& chassis_heading_state;
    /// @brief state name for the yaw axis
    const Cfg::StateName& yaw_state;
    /// @brief state name for the pitch axis
    const Cfg::StateName& pitch_state;

    /// @brief error monitor for chassis x axis estimates
    ErrorMonitor chassis_x_monitor;
    /// @brief error monitor for chassis y axis estimates
    ErrorMonitor chassis_y_monitor;
    /// @brief error monitor for chassis heading estimates
    ErrorMonitor chassis_heading_monitor;
    /// @brief error monitor for yaw gimbal estimates
    ErrorMonitor yaw_monitor;
    /// @brief error monitor for pitch gimbal estimates
    ErrorMonitor pitch_monitor;

    /// @brief position estimate to store position after integrating used for chassis odometry
    float pos_estimate[3] = { 0,0,0 };

    /// @brief previous pose to store the previous pose for chassis odometry
    float previous_pos[3] = { 0,0,0 };

public:
    /// @brief estimate the state of the gimbal
    /// @param estimator_config config data for this estimator
    /// @param sensor_manager reference to the sensor manager to get sensors
    /// @param can reference to the CAN manager to get motor objects so we can read directly from motors for odometry
    /// @param available_states vector of state names that are available
    GimbalAndChassisEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states);

    /// @copydoc Estimator::step_states
    void step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override) override;

    /// @copydoc Estimator::validate
    void validate(const RobotStateMap& updated_state_map) override;
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

    /// @brief radius of the flywheel in meters
    float flywheel_radius = 0;

    /// @brief flywheel motor left
    std::shared_ptr<Motor> flywheel_motor_left;
    /// @brief flywheel motor right
    std::shared_ptr<Motor> flywheel_motor_right;
    /// @brief state name for the ball exit velocity
    const Cfg::StateName& ball_exit_velocity;
    /// @brief monitor for flywheel estimate limits
    ErrorMonitor flywheel_monitor;

public:
    /// @brief make new flywheel estimator and set can data pointer and num states
    /// @param estimator_config config data for this estimator
    /// @param sensor_manager reference to the sensor manager to get sensors
    /// @param can reference to the CAN manager to get motor objects so we can read directly from motors for flywheel velocity
    /// @param available_states vector of state names that are available
    FlywheelEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states);

    /// @copydoc Estimator::step_states
    void step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override);

    /// @copydoc Estimator::validate
    void validate(const RobotStateMap& updated_state_map) override;
};

/// @brief Estimate the state of the feeder ball velocity based on the feeder encoder velocity.
struct FeederEstimator : public Estimator {
    private:
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

        /// @brief state name for the feeder ball velocity
        const Cfg::StateName& feeder_ball_state;
        /// @brief BuffEncoder on the feeder
        std::shared_ptr<BuffEncoder> feeder_encoder;
        /// @brief monitor for feeder estimate limits
        ErrorMonitor feeder_monitor;
    public:
        /// @brief Make new feeder estimator
        /// @param estimator_config config data for this estimator
        /// @param sensor_manager reference to the sensor manager to get sensors
        /// @param can reference to the CAN manager (not used)
        /// @param available_states vector of state names that are available
        FeederEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states);
    
        /// @copydoc Estimator::step_states
        void step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override) override;

        /// @copydoc Estimator::validate
        void validate(const RobotStateMap& updated_state_map) override;
};

/// @brief Estimate the state of the feeder ball velocity for the lower feeder based on the feeder encoder velocity. This is separate from the main FeederEstimator because it has different config data and we want to be able to disable one without affecting the other.
struct LowerFeederEstimator : public Estimator {
    private:
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

        /// @brief value for checking if the buff encoder spi is failing
        float reset_value = 0;
        /// @brief counts the number of time the buff encoder has reset from an spi issue
        int num_encoder_resets = 0;

        /// @brief state name for the feeder ball velocity
        const Cfg::StateName& feeder_ball_state;
        /// @brief BuffEncoder on the feeder
        std::shared_ptr<BuffEncoder> feeder_encoder;

        /// @brief monitor for lower feeder estimate limits
        ErrorMonitor lower_feeder_monitor;

        /// @brief feeder motor closer to the indexer
        std::shared_ptr<Motor> near_feeder_motor;
        /// @brief feeder motor farther from the indexer
        std::shared_ptr<Motor> far_feeder_motor;
    public:
        /// @brief Make new feeder estimator
        /// @param estimator_config config data for this estimator
        /// @param sensor_manager reference to the sensor manager to get sensors
        /// @param can reference to the CAN manager (not used)
        /// @param available_states vector of state names that are available
        LowerFeederEstimator(const Cfg::Estimator& estimator_config, SensorManager& sensor_manager, CANManager& can, std::vector<Cfg::StateName> available_states);
    
        /// @copydoc Estimator::step_states
        void step_states(RobotStateMap& updated_state_map, const RobotStateMap& previous_state_map, int override) override;

        /// @copydoc Estimator::validate
        void validate(const RobotStateMap& updated_state_map) override;
    };
