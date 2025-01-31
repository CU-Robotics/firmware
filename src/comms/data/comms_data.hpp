#pragma once

#include <cstdint>

namespace Comms {

enum class TypeLabel : uint8_t {
    NONE = 0x00,
    ExampleData = 0x01,
    TargetState = 0x55,
    OverrideState = 0x66,
    LoggingData = 0x77
};

enum class PhysicalMedium : uint8_t {
    HID,
    Ethernet
};

enum class Priority : uint8_t {
    High,
    Medium,
    Logging
};


/// @brief base class for all data structs that want to be sent over comms.
struct CommsData {
public:
    CommsData(TypeLabel type_label, PhysicalMedium physical_medium, Priority priority, uint16_t size) {
        this->type_label = type_label;
        this->physical_medium = physical_medium;
        this->priority = priority;
        this->size = size;
    }

    uint16_t size = 0;
    TypeLabel type_label : 8 = TypeLabel::NONE;

    PhysicalMedium physical_medium : 4 = PhysicalMedium::HID;
    Priority priority : 4 = Priority::High;
};

} // namespace Comms