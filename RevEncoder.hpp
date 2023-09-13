#include <cmath>
#include <FreqMeasureMulti.h>

#ifndef RevEncoder_H
#define RevEncoder_H


class RevEnc {
	private:
		uint8_t inPin;
		FreqMeasureMulti freq;
	public:
		RevEnc(uint8_t encPin);
        void read();
		float ticks, radians;
};

#endif