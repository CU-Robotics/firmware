#pragma once

// TODO: implement

#if defined(HIVE)

#include <vector>

#include "comms_data.hpp"
#include "utils/shapes/point.hpp"

#define NUM_SENSOR_VALUES 12
#define NUM_SENSORS 16
#define NUM_GAINS 24
#define NUM_ROBOT_CONTROLLERS 12
#define NUM_CAN_BUSES      2 // 2 cans per robot
#define NUM_MOTORS_PER_BUS 8 // 8 motors per can
#define NUM_MOTORS         (NUM_CAN_BUSES * NUM_MOTORS_PER_BUS)
#define STATE_LEN 24
#define NUM_ESTIMATORS 16

namespace Comms {


/// @brief TODO: blah
struct ConfigData {
    /// @brief robot id
    float robot;

    /// @brief matrix that defines type and neccessary values for each sensor
    float sensor_info[NUM_SENSORS][NUM_SENSOR_VALUES + 1];


    /// @brief gains matrix
    float gains[NUM_ROBOT_CONTROLLERS][NUM_GAINS];
    /// @brief gear ratio matrix
    float gear_ratios[NUM_ROBOT_CONTROLLERS][NUM_MOTORS];

    /// @brief matrix that contains the type, physical id, and physical bus of each motor
    float motor_info[NUM_MOTORS][3];
    /// @brief reference limits matrix
    float set_reference_limits[STATE_LEN][3][2];

    /// @brief the estimator id's and info
    float estimator_info[NUM_ESTIMATORS][STATE_LEN + 1];
    /// @brief controller id's and info
    float controller_info[NUM_ROBOT_CONTROLLERS][NUM_MOTORS + 1];
    /// @brief governor types
    float governor_types[STATE_LEN];
};

} // namespace Comms

#endif  // HIVE