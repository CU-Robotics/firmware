#pragma once

#include <cstdint>
#include "robot_state.hpp"
#include "../../controls/controller.hpp"

#define NUM_SENSOR_VALUES 12
#define NUM_SENSORS 16


namespace Comms {

enum class TypeLabel : uint8_t {
    // default type
    NONE = 0x00,

    // Hive types
    TargetState = 0x11,
    OverrideState = 0x22,
    LoggingData = 0x33,
    ExampleHive = 0x44,

    // Firmware types
    ExampleFW = 0x55,

    // TODO rest of types for final data structs
};

enum class PhysicalMedium : uint8_t {
    HID,
    Ethernet
};

enum class Priority : uint8_t {
    High,
    Medium,
};


/// @brief base class for all data structs that want to be sent over comms.
struct CommsData {
public:
    uint16_t size;
    TypeLabel type_label;
    Priority priority;
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

// Firmware struct types

struct FirmwareString : public CommsData {
    FirmwareString() {
        type_label = TypeLabel::ExampleFW;
        size = sizeof(FirmwareString);
        priority = Priority::Medium;
        memset(my_str, 0, 128);
    }
    char my_str[128];
};



// Hive struct types



// Megastruct defs
struct FirmwareData {
    // TODO: define structs for firmware outgoing data
};

struct HiveData  {
    ConfigData config_data;
    RobotState target_state;
    RobotState override_state;
};

} // namespace Comms