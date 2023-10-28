#include <FreqMeasureMulti.h>

#ifndef REV_ENCODER_H
#define REV_ENCODER_H

class RevEncoder {
	private:
		uint8_t pin;
		FreqMeasureMulti freq;
	public:
		RevEncoder(uint8_t pin);
		int getRawOutput();
		float getAngle();
};

#endif