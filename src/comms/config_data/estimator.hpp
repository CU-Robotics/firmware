#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
#include "comms_data.hpp" // for CommsData, TypeLabel, to_string
#include "state.hpp"      // for StateName
#include "motor.hpp"      // for MotorName
#include "sensor.hpp"     // for SensorName
#include "safety.hpp" // for assert_or_safety_mode

constexpr uint32_t MAX_GENERIC_STATE_USES_PER_ESTIMATOR = 16;
constexpr uint32_t MAX_GENERIC_MOTOR_USES_PER_ESTIMATOR = 16;
constexpr uint32_t MAX_GENERIC_SENSOR_USES_PER_ESTIMATOR = 16;

namespace Cfg {

enum class GenericSensorUse: uint32_t {
    UnsetSensorType,
    YawBuffEncoder,
    PitchBuffEncoder,
    FeederBuffEncoder,
    YawIcmImu,
};

enum class GenericEstimatorMotorUse : uint32_t {
    ChassisFrontRight,
    ChassisBackRight,
    ChassisBackLeft,
    ChassisFrontLeft,
    FlywheelLeft,
    FlywheelRight,
    Feeder,
};

enum class GenericEstimatorStateUse : uint32_t {
    ChassisX,
    ChassisY,
    ChassisHeading,
    GimbalYaw,
    GimbalPitch,
    ShooterBallVelocity,
    FeederBallPosition,
};

enum class EstimatorType : uint32_t {
    UnsetEstimatorType,
    GimbalAndChassis,
    FlywheelVelocity,
    FeederPosition,
    Actuators,
};

struct SensorInfo {
    float yaw_encoder_offset;
    float pitch_encoder_offset;
    float feeder_encoder_offset;
    float pitch_angle_at_imu_calibration;
    float yaw_start_angle;
    float pitch_start_angle;
    float roll_start_angle;
    float yaw_axis_vector[3];
    float pitch_axis_vector[3];
    float chassis_x_to_motor_rad;
    float chassis_y_to_motor_rad;
    float chassis_rad_to_motor_rad;
};

struct Estimator : Comms::CommsData {
    EstimatorType estimator_type;
    StateName generic_state_uses_to_names[MAX_GENERIC_STATE_USES_PER_ESTIMATOR];
    MotorName generic_motor_uses_to_names[MAX_GENERIC_MOTOR_USES_PER_ESTIMATOR];
    SensorName generic_sensor_uses_to_names[MAX_GENERIC_SENSOR_USES_PER_ESTIMATOR];
    SensorInfo sensor_info;

    /// @brief Get the motor name of this controller with the given generic use
    /// @param motor_use The generic use of the motor, as defined by the GenericControllerMotorUse enum
    /// @return The motor name with the given generic use, or an empty motor name if it was not found
    const MotorName& get_motor_name_by_generic_use(GenericEstimatorMotorUse motor_use) const {
        int motor_index = static_cast<int>(motor_use);
        safety::assert_or_safety_procedure(motor_index > MAX_GENERIC_MOTOR_USES_PER_ESTIMATOR || motor_index < 0, "Generic motor use index out of bounds");
        return generic_motor_uses_to_names[motor_index];
    }

    /// @brief Get the state name of this controller with the given generic use
    /// @param state_use The generic use of the state, as defined by the GenericControllerStateUse enum
    /// @return The state name with the given generic use, or an empty state name if it was not found
    const StateName& get_state_name_by_generic_use(GenericEstimatorStateUse state_use) const {
        int state_index = static_cast<int>(state_use);
        safety::assert_or_safety_procedure(state_index > MAX_GENERIC_STATE_USES_PER_ESTIMATOR || state_index < 0, "Generic state use index out of bounds");
        return generic_state_uses_to_names[state_index];
    }

    /// @brief Get the sensor name of this controller with the given generic use
    /// @param sensor_use The generic use of the sensor, as defined by the GenericControllerSensorUse enum
    /// @return The sensor name with the given generic use, or an empty sensor name if it was not found
    const SensorName& get_sensor_name_by_generic_use(GenericSensorUse sensor_use) const {
        int sensor_index = static_cast<int>(sensor_use);
        safety::assert_or_safety_procedure(sensor_index > MAX_GENERIC_SENSOR_USES_PER_ESTIMATOR || sensor_index < 0, "Generic sensor use index out of bounds");
        return generic_sensor_uses_to_names[sensor_index];
    }
};

}