#pragma once

#include <stdint.h>     // for uint8_t, uint32_t
#include "comms/data/comms_data.hpp" // for CommsData, TypeLabel,

namespace NewConfig {

struct ReferenceLimit {
    float min;
    float max;
};

enum class GovernorType : uint32_t {
    None,
    Position,
    Velocity,
    Acceleration,
};

struct Governor : Comms::CommsData {
    ReferenceLimit reference_limits;
    uint32_t governor_type;
    uint32_t state_name;

    Governor() : Comms::CommsData(Comms::TypeLabel::GovernorConfig, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(Governor)) {
        reference_limits.min = 0;
        reference_limits.max = 0;
        governor_type = None;
        state_name = UnsetStateName;
    }
};

}