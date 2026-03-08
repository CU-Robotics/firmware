#pragma once

#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/config_data/sensor.hpp"

struct StereoCamTriggerData : Comms::CommsData {
    StereoCamTriggerData() : CommsData(Comms::TypeLabel::StereoCamTriggerData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(StereoCamTriggerData)) { }
    StereoCamTriggerData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::StereoCamTriggerData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(StereoCamTriggerData)), camera_trigger_name(name) { }
    /// Sensor ID.
    Cfg::SensorName camera_trigger_name;

    /// State matching has not been designed yet.
};