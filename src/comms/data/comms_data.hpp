#pragma once

#include <stdint.h>     // for uintN_t
#include <string>       // for std::string

namespace Comms {

/// @brief TypeLabel is a unique identifier for each type of data that can be sent over comms.
enum class TypeLabel : uint8_t {
    NONE = 0x00,
    TestData,
    BigTestData,
    LoggingData,
    BuffEncoderData,
    ICMSensorData,
    RevEncoderData,
    TOFSensorData,
    LidarDataPacketSI,
    DR16Data,
    TempRobotState,
    TargetState,
    EstimatedState,
    OverrideState,
    ConfigSection,
    CommsRefData,
    LogData,
};

/// @brief Converts a TypeLabel to a string.
/// @param type_label The TypeLabel to convert.
/// @return The string representation of the TypeLabel.
inline std::string to_string(TypeLabel type_label) {
    switch (type_label) {
    case TypeLabel::NONE:
        return "NONE";
    case TypeLabel::TestData:
        return "TestData";
    case TypeLabel::BigTestData:
        return "BigTestData";
    case TypeLabel::LoggingData:
        return "LoggingData";
    case TypeLabel::BuffEncoderData:
        return "BuffEncoderData";
    case TypeLabel::ICMSensorData:
        return "ICMSensorData";
    case TypeLabel::RevEncoderData:
        return "RevEncoderData";
    case TypeLabel::TOFSensorData:
        return "TOFSensorData";
    case TypeLabel::LidarDataPacketSI:
        return "LidarDataPacketSI";
    case TypeLabel::DR16Data:
        return "DR16Data";
    case TypeLabel::TempRobotState:
        return "TempRobotState"; 
    case TypeLabel::TargetState:
        return "TargetState";
    case TypeLabel::EstimatedState:
        return "EstimatedState";
    case TypeLabel::OverrideState:
        return "OverrideState";
    case TypeLabel::ConfigSection:
        return "ConfigSection";
    case TypeLabel::CommsRefData:
        return "CommsRefData";
    case TypeLabel::LogData:
        return "LogData";
    // no default case, so the compiler will warn us if we forget a case
    }

    // because no default case, this gets rid of the no return warning
    return "UNKNOWN";
}

/// @brief PhysicalMedium is the medium over which the data is sent.
enum class PhysicalMedium : uint8_t {
    HID = 0x00,
    Ethernet,
};

/// @brief Priority is the priority of the data being sent.
enum class Priority : uint8_t {
    High = 0x00,
    Medium,
    Logging,
};

/// @brief base class for all data structs that want to be sent over comms.
struct CommsData {
public:
    /// @brief Primary constructor, initializes all fields.
    /// @param type_label The type of data being sent.
    /// @param physical_medium The medium over which the data is sent.
    /// @param priority The priority of the data being sent.
    /// @param size The size of the data in bytes.
    CommsData(TypeLabel type_label, PhysicalMedium physical_medium, Priority priority, uint16_t size) {
        this->type_label = type_label;
        this->physical_medium = physical_medium;
        this->priority = priority;
        this->size = size;
    }

    /// @brief size of the data in bytes
    uint16_t size : 16 = 0;
    /// @brief type of data
    TypeLabel type_label : 8 = TypeLabel::NONE;
    /// @brief medium over which the data is sent
    PhysicalMedium physical_medium : 4 = PhysicalMedium::Ethernet;
    /// @brief priority of the data
    Priority priority : 4 = Priority::Medium;
};

} // namespace Comms
