/**
 * @file rev_encoder.hpp
 * @brief The header file for the Rev Encoder firmware
 * @date 2023-10-31
 */

#include <cmath>
#include <FreqMeasureMulti.h>

#ifndef REV_ENCODER_H
#define REV_ENCODER_H

class RevEncoder 
{
	private:
		/**
		* @brief the pin number that the encoders signal pin is plugged into
		*/
		uint8_t inPin;
		/**
		* @brief Used to read rise time of the encoder
		*/
		FreqMeasureMulti freq;
		/**
		 * @brief how many readings will be avergaed to update the current angle 
		 */
		const int READ_SIZE = 10;
		/**
		 * @brief used to add up sequencial encoder readings to evenutually average
		 */
		float sum;
		/** 
		 * @brief to track how many encoder readings are used to eventually average 
		 */
		int count;
		/**
		* @brief measure of current angle in ticks (1024 ticks = 1 rotation). When set to 1 individual readings are returned
		*/
		float ticks;
		/**
		* @brief measure of current angle in radians 
		*/
		float radians;
	public:
		/**
		* @brief Construct a new rev_encoder object
		* @param encPin the pin number that the encoders signal pin is plugged into
		* @param baudrate the desired baudrate for the encoder
		*/
		RevEncoder(uint8_t encPin, int baudrate);
		/**
		* @brief updates the current angle in ticks (1024 = 1 Rotation) and radians.
		*/
		void read();
		/**
		* @brief return the last angle of the encoder in ticks (1024 = 1 Rotation).
		*/
		float getAngleTicks();
		/**
		* @brief return the last angle of the encoder in radians
		*/
		float getAngleRadians();
};

#endif