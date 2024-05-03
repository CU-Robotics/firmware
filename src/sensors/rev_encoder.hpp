#include <cmath>
#include <FreqMeasureMulti.h>

#ifndef REV_ENCODER_H
#define REV_ENCODER_H

/// @brief the class for the Rev Through Bore Encoder(www.revrobotics.com/rev-11-1271/)
class RevEncoder {
private:
	/// @brief the pin number that the encoder's signal pin is plugged into
	uint8_t in_pin;
	/// @brief Used to read rise time of the encoder
	FreqMeasureMulti freq;
	/// @brief measure of current angle in ticks [0, 1023]
	int ticks;
	/// @brief measure of current angle in radians [0, 2pi)
	float radians;
	/// @brief the starting value of the encoder in radians
	float starting_value = 0;
public:
	/// @brief Construct a new rev_encoder object without initializing the encoder
	RevEncoder() {};

	/// @brief Construct a new rev_encoder object
	/// @param encoder_pin the pin number that the encoders signal pin is plugged into
	RevEncoder(uint8_t encoder_pin);

	/// @brief initialize the encoder with the correct pin
	/// @param encoder_pin the pin number that the encoders signal pin is plugged into
	/// @param is_relative if the encoder is relative or absolute
	void init(uint8_t encoder_pin, bool is_relative);

	/// @brief updates ticks and radians to the current angle 
	void read();
	/// @brief get the last angle of the encoder in ticks
	/// @return the last angle of the encoder in ticks [0, 1023]
	float get_angle_ticks();
	/// @brief get the last angle of the encoder in radians
	/// @return the last angle of the encoder in radians [0, 2pi)
	float get_angle_radians();
};

#endif