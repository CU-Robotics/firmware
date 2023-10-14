/**
 * @file rev_encoder.hpp
 * @author Aidan Donnelly
 * @brief Header file for the Rev Encoder 
 * @version 0.1
 * @date 2023-10-14
 */
#include <cmath>
#include <FreqMeasureMulti.h>

#ifndef RevEncoder_H
#define RevEncoder_H

class RevEncoder 
{
	private:
		uint8_t inPin;
		FreqMeasureMulti freq;
	public:
		RevEncoder(uint8_t encPin);
        void read();
		float ticks, radians;
};

#endif