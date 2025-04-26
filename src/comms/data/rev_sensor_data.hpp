#pragma once 

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>                             // uintN_t

/// @brief Structure for the Rev encoder sensor.
struct RevSensorData : Comms::CommsData {
    RevSensorData() : CommsData(Comms::TypeLabel::RevEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(RevSensorData)) { }
	/// Sensor ID.
	uint8_t id;
	/// Encoder ticks.
	int ticks;
	/// Angle in radians.
	float radians;
};