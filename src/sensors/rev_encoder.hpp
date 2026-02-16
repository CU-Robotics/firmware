#pragma once

#include <cmath>
#include <FreqMeasureMulti.h>
#include "comms/data/rev_sensor_data.hpp"

/// @brief the class for the Rev Through Bore Encoder(www.revrobotics.com/rev-11-1271/)
class RevEncoder {
private:
	const NewConfig::RevEncoder& config;
	/// @brief Used to read rise time of the encoder
	FreqMeasureMulti freq;
	/// @brief measure of current angle in ticks [0, 1023]
	int ticks;
	/// @brief measure of current angle in radians [0, 2pi)
	float radians;
	/// @brief the starting value of the encoder in radians
	float starting_value = 0;
public:
	/// @brief Construct a new rev_encoder object
	/// @param encoder_pin the pin number that the encoders signal pin is plugged into
	RevEncoder(const NewConfig::RevEncoder& config);

	/// @brief initialize the encoder with the correct pin
	/// @param encoder_pin the pin number that the encoders signal pin is plugged into
	/// @param is_relative if the encoder is relative or absolute
	void init();

	/// @brief updates ticks and radians to the current angle 
	/// @return true if successful, false if no data available
	bool read();
	
	/// @brief get the last angle of the encoder in ticks
	/// @return the last angle of the encoder in ticks [0, 1023]
	float get_angle_ticks();
	/// @brief get the last angle of the encoder in radians
	/// @return the last angle of the encoder in radians [0, 2pi)
	float get_angle_radians();

	RevEncoderData get_data_for_comms();

	/// @brief print the encoder details
	void print();
};

#endif