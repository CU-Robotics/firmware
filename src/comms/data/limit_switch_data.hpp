#pragma once

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

struct LimitSwitchData : Comms::CommsData {
    LimitSwitchData() : CommsData(Comms::TypeLabel::LimitSwitchData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(LimitSwitchData)) { }
    LimitSwitchData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::LimitSwitchData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(LimitSwitchData)), switch_name(name) { }
    Cfg::SensorName switch_name;
    uint8_t is_pressed;


    void print() const {
        printf("LimitSwitchData - switch_name: %lu, is_pressed: %u\n", static_cast<uint32_t>(switch_name), is_pressed);
    }
};