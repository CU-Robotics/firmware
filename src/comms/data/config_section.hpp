#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

// TODO: make this ethernet capable
/// @brief Section of a config packet
struct ConfigSection : Comms::CommsData {
    ConfigSection() : CommsData(Comms::TypeLabel::ConfigSection, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(ConfigSection)) { }

    /// @brief filler byte
    uint8_t filler = 0xff;
    /// @brief Section ID
    int8_t section_id = 0;
    /// @brief Subsection ID
    int8_t subsection_id = 0;
    /// @brief Info bit, stores the config request bit
    bool request_bit = 0;

    /// @brief Size of the whole section
    uint16_t section_size = 0;
    /// @brief Size of the subsection
    uint16_t subsection_size = 0;

    /// @brief Raw config data
    uint8_t raw[1000] = { 0 };
};