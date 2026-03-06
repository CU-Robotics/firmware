#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t
#include "comms/config_data/sensor.hpp"

/// @brief Structure for the Rev encoder sensor.
struct RevSensorData : Comms::CommsData {
    RevSensorData() : CommsData(Comms::TypeLabel::RevEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(RevSensorData)) { }
	RevSensorData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::RevEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(RevSensorData)), encoder_name(name) { }
	Cfg::SensorName encoder_name;
	/// Encoder ticks.
	int ticks;
	/// Angle in radians.
	float radians;
};