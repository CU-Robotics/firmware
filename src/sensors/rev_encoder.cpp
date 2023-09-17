#include "rev_encoder.hpp"

RevEnc::RevEnc(uint8_t pin) {
    this->pin = pin;
	freq.begin(this->inPin, FREQMEASUREMULTI_MARK_ONLY);
}

int RevEnc::getAngleRaw() {
	while (this->freq.available() > 1) this->freq.read(); // Burn through buffer of values in freq
	int angle = round(freq.countToNanoseconds(freq.read()) / 1000.0);
	if (angle >= 1 && angle <= 1025) return angle;
	return -1;
}

float RevEnc::getAngle() {
	int rawAngle = this->getAngleRaw();
	return rawAngle == -1 ? -1 : map(rawAngle, 1, 1024, -PI*1000.0, PI*1000.0) / 1000.0;
}