#pragma once

#include <cstdint>          // for uintN_t

namespace Comms {

/// @brief TypeLabel is a unique identifier for each type of data that can be sent over comms.
enum class TypeLabel : uint8_t {
    NONE = 0x00,
    TestData,
    LoggingData,
};

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
    uint16_t size : 16;
    /// @brief type of data
    TypeLabel type_label : 8;
    /// @brief medium over which the data is sent
    PhysicalMedium physical_medium : 4;
    /// @brief priority of the data
    Priority priority : 4;
};

} // namespace Comms
