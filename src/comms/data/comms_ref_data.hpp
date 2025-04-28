#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

// TODO: make this nice
/// @brief Ref data
struct CommsRefData : Comms::CommsData {
    CommsRefData() : CommsData(Comms::TypeLabel::CommsRefData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(CommsRefData)) { }

    /// @brief Raw data
    uint8_t raw[180] = { 0 };
};