#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData

#include <stdint.h>                             // uintN_t
#include "comms/config_data/sensor.hpp"

/// @brief Structure for the Rev encoder sensor.
struct RevSensorData : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the RevSensorData struct.
    RevSensorData() : CommsData(Comms::TypeLabel::RevEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(RevSensorData)) { }
	/// @brief Constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the RevSensorData struct, and also sets the encoder name.
	/// @param name The name of the encoder that this data corresponds to.
	RevSensorData(Cfg::SensorName name) : CommsData(Comms::TypeLabel::RevEncoderData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(RevSensorData)), encoder_name(name) { }
	/// @brief The name of the encoder that this data corresponds to.
	Cfg::SensorName encoder_name = Cfg::SensorName::UnsetSensorName;
	/// @brief Encoder ticks.
	int ticks = 0;
	/// @brief Angle in radians.
	float radians = 0.0;

	/// @brief Print the Rev encoder data to the serial console for debugging purposes.
	void print() const {
		printf("RevSensorData - encoder_name: %lu, ticks: %d, radians: %f\n", static_cast<uint32_t>(encoder_name), ticks, radians);
	}
};