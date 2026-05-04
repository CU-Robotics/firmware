#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
// #include "comms_data.hpp" // for CommsData, TypeLabel, to_string
#include "state.hpp"      // for StateName
#include "motor.hpp"      // for MotorName
#include "sensor.hpp"     // for SensorName
#include "safety.hpp" // for assert_or_safety_mode

/// These values are arbitrary limits for the size of the arrays in the Estimator struct. 
// They should be large enough to accommodate any reasonable number of states, motors, or sensors used by an estimator configuration.

/// @brief The max number of generic state uses per estimator.
constexpr uint32_t MAX_GENERIC_STATE_USES_PER_ESTIMATOR = 16;
/// @brief The max number of generic motor uses per estimator.
constexpr uint32_t MAX_GENERIC_MOTOR_USES_PER_ESTIMATOR = 16;
/// @brief The max number of generic sensor uses per estimator.
constexpr uint32_t MAX_GENERIC_SENSOR_USES_PER_ESTIMATOR = 16;

namespace Cfg {
/// @brief This enum represents the different generic uses for sensors that an estimator can have. These are used to map the specific sensors on the robot to their generic uses in the estimator config.
///
/// If multiple of the same estimator is needed for some reason then the generic uses can be used to differentiate between the different sensors that the estimators are using.
/// For example, if two Feeder Estimators are needed then the generic sensor uses can be used to differentiate between the encoder used on the first Feeder Estimator and the encoder used on the second Feeder Estimator.
/// You would do this in the yaml by doing something like
/// Estimators:
///   Feeder Position:
///     Sensors Used: {Feeder Buff Encoder: "Feeder Encoder 1"}
///   Feeder Position:
///     Sensors Used: {Feeder Buff Encoder: "Feeder Encoder 2"}
/// In this example, both Feeder Estimators are using a sensor that has the generic use of `FeederBuffEncoder`, but they are differentiated by the specific sensor name that they are mapped to in the config.
enum class GenericSensorUse: uint32_t {
    YawBuffEncoder,
    PitchBuffEncoder,
    FeederBuffEncoder,
    YawIcmImu,
};

/// @brief This enum represents the different generic uses for motors that an estimator can have. These are used to map the specific motors on the robot to their generic uses in the estimator config.
///
/// If multiple of the same estimator is needed for some reason then the generic uses can be used to differentiate between the different motors that the estimators are using.
/// For example, if two Feeder Estimators are needed then the "generic motor uses" can be used to differentiate between the motors used on the first Feeder Estimator and the motors used on the second Feeder Estimator.
/// You would do this in the yaml by doing something like
/// Estimators:
///   Feeder Position:
///     Motors Used: {Feeder: "Feeder Motor 1"}
///   Feeder Position:
///     Motors Used: {Feeder: "Feeder Motor 2"}
/// In this example, both Feeder Estimators are using a motor that has the generic use of `Feeder`, but they are differentiated by the specific motor name that they are mapped to in the config.
enum class GenericEstimatorMotorUse : uint32_t {
    ChassisFrontRight,
    ChassisBackRight,
    ChassisBackLeft,
    ChassisFrontLeft,
    FlywheelLeft,
    FlywheelRight,
    Feeder,
    FeederClose,
    FeederFar,
};

/// @brief This enum represents the different generic uses for states that an estimator can have. These are used to map the specific states on the robot to their generic uses in the estimator config.
///
/// If multiple of the same estimator is needed for some reason then the generic uses can be used to differentiate between the different states that the estimators are estimating.
/// For example, if two Feeder Estimators are needed then the "generic state uses" can be used to differentiate between the states estimated on the first Feeder Estimator and the states estimated on the second Feeder Estimator.
/// You would do this in the yaml by doing something like
/// Estimators:
///   Feeder Position:
///     Estimated States: {FeederBallPosition: "Feeder 1"}
///   Feeder Position:
///     Estimated States: {FeederBallPosition: "Feeder 2"}
/// In this example, both Feeder Estimators are using a state that has the generic use of `FeederBallPosition`, but they are differentiated by the specific state name that they are mapped to in the config.
enum class GenericEstimatorStateUse : uint32_t {
    ChassisX,
    ChassisY,
    ChassisHeading,
    GimbalYaw,
    GimbalPitch,
    ShooterBallVelocity,
    FeederBallPosition,
    LowerFeederBallPosition,
};

/// @brief The different types of estimators that can be configured in the config.
enum class EstimatorType : uint32_t {
    UnsetEstimatorType,
    GimbalAndChassis,
    FlywheelVelocity,
    FeederPosition,
    LowerFeederPosition,
};
/// @brief The `SensorInfo` struct contains all the sensor related information for the estimator config.
// This includes the offsets for the encoders, the ratios and directions for the feeder, etc.
struct SensorInfo {
    /// @brief the offset for the yaw encoder used in the gimbal and chassis estimator, in radians. This is used to convert the raw encoder values to the actual angles of the gimbal and chassis.
    float yaw_encoder_offset = 0.0;
    /// @brief The pitch encoder offset used in the gimbal and chassis estimator, in radians. This is used to convert the raw encoder values to the actual angles of the gimbal and chassis.
    float pitch_encoder_offset = 0.0;
    /// @brief The feeder encoder offset used in the feeder position estimator, in radians. This is used to convert the raw encoder values to the actual angle of the feeder spindexer
    float feeder_encoder_offset = 0.0;
    /// @brief The ratio between the feeder spindexer angle and the amount of balls fed. This is typically calculated as (number of balls fed per revolution of the spindexer) / (2 * PI)
    float feeder_ratio = 0.0;
    /// @brief the direction for the feeder, used to determine the sign of the encoder values.
    float feeder_direction = 0.0;
    /// @brief the pitch angle at IMU calibration, in radians.
    float pitch_angle_at_imu_calibration = 0.0;
    /// @brief the start angle for the yaw, in radians
    float yaw_start_angle = 0.0;
    /// @brief the start angle for the pitch, in radians
    float pitch_start_angle = 0.0;
    /// @brief the start angle for the roll, in radians
    float roll_start_angle = 0.0;
    /// @brief an average reading of the 3 imu axis gyro values during a calibration where the yaw is spun freely.
    float yaw_axis_vector[3] = { 0.0, 0.0, 0.0 };
    /// @brief an average reading of the 3 imu axis gyro values during a calibration where the pitch is dropped freely from its top position.
    float pitch_axis_vector[3] = { 0.0, 0.0, 0.0 };
    /// @brief the direction of the pitch encoder
    float pitch_encoder_direction;
    /// @brief the direction of the yaw encoder
    float yaw_encoder_direction;
    /// @brief whether or not the imu is mounted on the pitch
    uint32_t has_pitch_imu;
    /// @brief the ratio between chassis x velocity in m/s and motor velocity in rad/s for the chassis motors, used by the X Drive estimator.
    float chassis_x_to_motor_rad = 0.0;
    /// @brief the ratio between chassis y velocity in m/s and motor velocity in rad/s for the chassis motors, used by the X Drive estimator.
    float chassis_y_to_motor_rad = 0.0;
    /// @brief the ratio between chassis angular velocity in rad/s and motor velocity in rad/s for the chassis motors, used by the X Drive estimator.
    float chassis_rad_to_motor_rad = 0.0;
    /// @brief the radius of the flywheel, in meters.
    float flywheel_radius = 0.0;
    /// @brief How much weight the estimator should put on the flywheel motor velocity when estimating the shooter ball velocity vs the velocity of the ball given from the referee system.
    float flywheel_motor_estimate_weight = 0.0;
};

/// @brief The `Estimator` struct represents the configuration for an estimator, including its type, the generic uses for states, motors and sensors, and the sensor info.
struct Estimator : Comms::CommsData {
    /// @brief Type of the estimator, as defined by the EstimatorType enum.
    EstimatorType estimator_type = EstimatorType::UnsetEstimatorType;
    /// @brief The specific state names that this estimator estimates, indexed by their generic use as defined by the GenericEstimatorStateUse enum
    StateName generic_state_uses_to_names[MAX_GENERIC_STATE_USES_PER_ESTIMATOR] = { StateName::UnsetStateName };
    /// @brief The specific motor names that this estimator uses, indexed by their generic use as defined by the GenericControllerMotorUse enum
    MotorName generic_motor_uses_to_names[MAX_GENERIC_MOTOR_USES_PER_ESTIMATOR] = { MotorName::UnsetMotorName };
    /// @brief The specific sensor names that this estimator uses, indexed by their generic use as defined by the GenericControllerSensorUse enum
    SensorName generic_sensor_uses_to_names[MAX_GENERIC_SENSOR_USES_PER_ESTIMATOR] = { SensorName::UnsetSensorName };
    /// @brief The sensor info for this estimator, which contains all the sensor related information for the estimator config.
    SensorInfo sensor_info;

    /// @brief Get the motor name of this controller with the given generic use
    /// @param motor_use The generic use of the motor, as defined by the GenericControllerMotorUse enum
    /// @return The motor name with the given generic use, or an empty motor name if it was not found
    const MotorName& get_motor_name_by_generic_use(GenericEstimatorMotorUse motor_use) const {
        int motor_index = static_cast<int>(motor_use);

        safety::assert_or_safety_procedure(motor_index < (int)MAX_GENERIC_MOTOR_USES_PER_ESTIMATOR && motor_index >= 0, "Generic motor use index out of bounds");
        return generic_motor_uses_to_names[motor_index];
    }

    /// @brief Get the state name of this controller with the given generic use
    /// @param state_use The generic use of the state, as defined by the GenericControllerStateUse enum
    /// @return The state name with the given generic use, or an empty state name if it was not found
    const StateName& get_state_name_by_generic_use(GenericEstimatorStateUse state_use) const {
        int state_index = static_cast<int>(state_use);
        for(uint32_t i = 0; i < MAX_GENERIC_STATE_USES_PER_ESTIMATOR; i++) {
        }
        safety::assert_or_safety_procedure(state_index < (int)MAX_GENERIC_STATE_USES_PER_ESTIMATOR && state_index >= 0, "Generic state use index out of bounds");
        return generic_state_uses_to_names[state_index];
    }

    /// @brief Get the sensor name of this controller with the given generic use
    /// @param sensor_use The generic use of the sensor, as defined by the GenericControllerSensorUse enum
    /// @return The sensor name with the given generic use, or an empty sensor name if it was not found
    const SensorName& get_sensor_name_by_generic_use(GenericSensorUse sensor_use) const {
        int sensor_index = static_cast<int>(sensor_use);
        safety::assert_or_safety_procedure(sensor_index < (int)MAX_GENERIC_SENSOR_USES_PER_ESTIMATOR && sensor_index >= 0, "Generic sensor use index out of bounds");
        return generic_sensor_uses_to_names[sensor_index];
    }
};

}