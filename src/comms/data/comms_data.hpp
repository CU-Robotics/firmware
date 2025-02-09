#pragma once

#include <cstdint>
#include "../../controls/controller.hpp"

#define NUM_SENSOR_VALUES 12
#define NUM_SENSORS 16

// RobotState stuff
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



namespace Comms {


/// @brief Label for a particular piece of data derived from CommsData.
enum class TypeLabel : uint8_t {
    // default type
    NONE = 0x00,

    // Hive types
    TargetState = 0x11,
    OverrideState = 0x22,
    LoggingData = 0x33,

    // Firmware types
    FWSample1 = 0x77,
    FWSample2 = 0x88,
    FWSample3 = 0x99,

    // TODO rest of types for final data structs
};

/// @brief Label to describe the physical implementation a piece of data needs to go through.
enum class PhysicalMedium : uint8_t {
    HID,
    Ethernet
};

/// @brief Label to describe the priority of a piece of data (determines what order it will be sent in with regards to everything else)
enum class Priority : uint8_t {
    High,
    Medium,
};

/// @brief Base class that all data that is sent over comms must inherit from.
struct CommsData {
public:
    /// @brief Generic constructor for unspecified CommsData object
    CommsData() {
        CommsData(sizeof(CommsData), TypeLabel::NONE, Priority::High);
    }
    /// @brief General constructor for CommsData with a given type, priority, and size
    /// @param size Size of derived object
    /// @param type_label Type of derived object
    /// @param priority Priority of transmitting derived object
    CommsData(uint16_t size, TypeLabel type_label, Priority priority) {
        this->type_label = type_label;
        // this->physical_medium = physical_medium;
        this->priority = priority;
        this->size = size;
    }

    /// @brief Size of this object
    uint16_t size = 0;

    /// @brief Type label for this object
    TypeLabel type_label : 8;

    // PhysicalMedium physical_medium : 4 = PhysicalMedium::HID;

    /// @brief Priority of this object
    Priority priority : 4;
};


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

// Hive struct types

/// @brief Struct containing the full robot state including time
struct RobotState : public CommsData{
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



// Firmware struct types

/// @brief Sample data struct for FirmwareData
struct FWSample1 : public CommsData {
    /// @brief constructor initializing CommsData
    FWSample1() : CommsData(sizeof(FWSample1), TypeLabel::FWSample1, Priority::Medium) { }

    /// @brief Sample number
    uint32_t num;
};

/// @brief Sample data struct for FirmwareData
struct FWSample2 : public CommsData {
    /// @brief constructor initializing CommsData
    FWSample2() : CommsData(sizeof(FWSample2), TypeLabel::FWSample2, Priority::Medium) { }

    /// @brief Sample number
    uint32_t num;
};

/// @brief Sample data struct for FirmwareData
struct FWSample3 : public CommsData {
    /// @brief constructor initializing CommsData
    FWSample3() : CommsData(sizeof(FWSample3), TypeLabel::FWSample3, Priority::Medium) { }

    /// @brief Sample number
    uint32_t num;
};


// Megastruct defs

/// @brief Object containing all of the data types that are sent over comms, which specifically come from Firmware and go to Hive.
struct FirmwareData {
    // TODO: define structs for firmware outgoing data

    /// @brief Sample data 1
    FWSample1 sample1;

    /// @brief Sample data 2
    FWSample2 sample2;

    /// @brief Sample data 3
    FWSample3 sample3;
};

/// @brief Object containing all of the data types that are sent over comms, which specifically come from Hive and go to Firmware.
struct HiveData {
    /// @brief Target state data
    RobotState target_state;

    /// @brief Override state data
    RobotState override_state;
};


} // namespace Comms