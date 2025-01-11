#pragma once

// copied from hive to mimic its implementation of robot state data

/* C++ includes */
#include <atomic>
#include <cstring>

// Robot state constants

/// @brief Number of state
constexpr int	ROBOT_STATE_LENGTH = 24u;
/// @brief Number of types of state (position, velocity, acceleration, ...)
constexpr int	ROBOT_STATE_DERIVATIVES_COUNT = 3u;

/// @brief Index of position in robot state
constexpr int 	ROBOT_STATE_POSITION = 0u;
/// @brief Index of velocity in robot state
constexpr int 	ROBOT_STATE_VELOCITY = 1u;
/// @brief Index of acceleration in robot state
constexpr int 	ROBOT_STATE_ACCELERATION = 2u;

/// @brief Struct containing the full robot state including time
struct RobotState {
    /// @brief Default constructor
    RobotState() = default;

    /// @brief Construct a RobotState object with raw byte arrays for time and state
    /// @param raw_time Raw byte array representing time
    /// @param raw_state Raw byte array representing state
    RobotState(char* raw_time, char* raw_state) {
        memcpy(&time, raw_time, sizeof(double));
        memcpy(&state, raw_state, sizeof(RobotState::state));
    }

    // RobotState(const RobotState &robot_state){
    // 	memcpy(&time,  &robot_state.time, sizeof(double));
    // 	memcpy(state, robot_state.state, sizeof(RobotState::state));
    // }
    /// @brief Time of the teensy
    double time = 0.0;
    /// @brief Full robot state array
    float state[ROBOT_STATE_LENGTH][ROBOT_STATE_DERIVATIVES_COUNT] = { 0 };
    /// @brief The delay in communication between the teensy and the khadas
    double comms_delay = 0;
};


// Vector3 definitions for interpreting robot state 
struct Vector3 {
    Vector3();
    Vector3(RobotState robot_state, int derivative);
    float x;
    float y;
    float z;
};

Vector3::Vector3() {
    x = 0;
    y = 0;
    z = 0;
}

Vector3::Vector3(RobotState robot_state, int derivative) {
    derivative = derivative % ROBOT_STATE_DERIVATIVES_COUNT;    // wrap it to be in bounds. better to just check this but this is quick to implement

    x = robot_state.state[0][derivative];     // x located at [0]
    y = robot_state.state[1][derivative];     // y located at [1]
    z = robot_state.state[2][derivative];     // z located at [2]
}
