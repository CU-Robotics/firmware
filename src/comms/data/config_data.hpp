#pragma once

#include <vector>

#include "comms_data.hpp"
#include "../../controls/controller.hpp"

#define NUM_SENSOR_VALUES 12
#define NUM_SENSORS 16

namespace Comms {


// literally JUST a config object right now. 
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