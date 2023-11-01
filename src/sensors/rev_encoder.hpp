#include <cmath>
#include <FreqMeasureMulti.h>

#ifndef REV_ENCODER_H
#define REV_ENCODER_H

/// @brief the class for the Rev Encoder
class RevEncoder 
{
	private:
		/// @brief the pin number that the encoders signal pin is plugged into
		uint8_t inPin;
		/// @brief Used to read rise time of the encoder
		FreqMeasureMulti freq;
		//// @brief measure of current angle in ticks (1024 ticks = 1 rotation). When set to 1 individual readings are returned
		float ticks;
		/// @brief measure of current angle in radians 
		float radians;
	public:
		/// @brief Construct a new rev_encoder object
		/// @param encPin the pin number that the encoders signal pin is plugged into
		/// @param baudrate the desired baudrate for the encoder
		RevEncoder(uint8_t encPin, int baudrate);
		/// @brief updates the current angle in ticks (1024 = 1 Rotation) and radians.
		void read();
		/// @brief get the last angle of the encoder ticks
		/// @return the last angle of the encoder in ticks (1024 = 1 Rotation).
		float getAngleTicks();
		/// @brief get the last angle of the encoder in radians
		/// @return the last angle of the encoder in radians
		float getAngleRadians();
};

#endif