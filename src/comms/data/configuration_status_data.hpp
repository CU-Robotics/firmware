#pragma once

#include "comms/data/comms_data.hpp"
#include <stdint.h>

struct ConfigurationStatusData : Comms::CommsData{
    ConfigurationStatusData() : CommsData(Comms::TypeLabel::ConfigurationStatus, Comms::PhysicalMedium::HID, Comms::Priority::High, sizeof(CommsRefData)) { } 
    uint32_t ready_for_config = 0;
    uint32_t is_configured = 0;
};