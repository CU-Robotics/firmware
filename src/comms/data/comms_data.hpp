#pragma once
#include <stdint.h>     // for uintN_t
#include <string>       // for std::string

namespace Comms {


    /// @brief TypeLabel is a unique identifier for each type of data that can be sent over comms.
enum class TypeLabel : uint16_t {
    NONE = 0x00,
    TestData,
    BigTestData,
    BuffEncoderData,
    RevEncoderData,
    ICMSensorData,
    LsmSensorData,
    LidarDataPacketSI,
    LimitSwitchData,
    StereoCamTriggerData,
    DR16Data,
    ET16SData,
    MotorStateData,
    TargetState,
    EstimatedState,
    OverrideState,
    CommsRefData,
    ConfigStart,
    ConfigurationStatus,
    ControllerConfig,
    EstimatorConfig,
    MotorConfig,
    BuffEncoderConfig,
    IcmImuConfig,
    LsmImuConfig,
    D200LidarConfig,
    StereoCameraTriggerConfig,
    StateConfig,
    TransmitterConfig,
    LoggingData,
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
    case TypeLabel::BuffEncoderData:
        return "BuffEncoderData";
    case TypeLabel::ICMSensorData:
        return "ICMSensorData";
    case TypeLabel::LsmSensorData:
        return "LsmSensorData";
    case TypeLabel::RevEncoderData:
        return "RevEncoderData";
    case TypeLabel::LidarDataPacketSI:
        return "LidarDataPacketSI";
    case TypeLabel::LimitSwitchData:
        return "LimitSwitchData";
    case TypeLabel::StereoCamTriggerData:
        return "StereoCamTriggerData";
    case TypeLabel::DR16Data:
        return "DR16Data";
    case TypeLabel::ET16SData:
        return "ET16SData";
    case TypeLabel::MotorStateData:
        return "MotorStateData";
    case TypeLabel::LoggingData:
        return "LoggingData";
    case TypeLabel::TargetState:
        return "TargetState";
    case TypeLabel::EstimatedState:
        return "EstimatedState";
    case TypeLabel::OverrideState:
        return "OverrideState";

    case TypeLabel::CommsRefData:
        return "CommsRefData";
    case TypeLabel::ConfigStart:
        return "ConfigStart";
    case TypeLabel::ConfigurationStatus:
        return "ConfigurationStatus";
    case TypeLabel::ControllerConfig:
        return "ControllerConfig";
    case TypeLabel::EstimatorConfig:
        return "EstimatorConfig";
    case TypeLabel::MotorConfig:
        return "MotorConfig";
    case TypeLabel::BuffEncoderConfig:
        return "BuffEncoderConfig";
    case TypeLabel::IcmImuConfig:
        return "IcmImuConfig";
    case TypeLabel::LsmImuConfig:
        return "LsmImuConfig";
    case TypeLabel::D200LidarConfig:
        return "D200LidarConfig";
    case TypeLabel::StereoCameraTriggerConfig:
        return "StereoCameraTriggerConfig";
    case TypeLabel::StateConfig:
        return "StateConfig";
    case TypeLabel::TransmitterConfig:
        return "TransmitterConfig";
    // no default case, so the compiler will warn us if we forget a case
    }

    // because no default case, this gets rid of the no return warning
    return "UNKNOWN";
}

/// @brief PhysicalMedium is the medium over which the data is sent.
enum class PhysicalMedium : uint16_t {
    HID = 0x00,
    Ethernet,
};

/// @brief Priority is the priority of the data being sent.
enum class Priority : uint16_t {
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

    CommsData() = default;

    /// @brief size of the data in bytes
    uint16_t size : 16 = 0;
    /// @brief type of data
    TypeLabel type_label : 16 = TypeLabel::NONE;
    /// @brief medium over which the data is sent
    PhysicalMedium physical_medium : 16 = PhysicalMedium::Ethernet;
    /// @brief priority of the data
    Priority priority : 16 = Priority::Medium;
};

} // namespace Comms
