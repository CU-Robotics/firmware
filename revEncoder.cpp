#include <cmath>
#include <FreqMeasureMulti.h>


class RevEncoder {
    public: 
        float ticks, radians; 
    private:
        FreqMeasureMulti freq;
        int inPin;

    // inPin: 
    RevEncoder::RevEncoder(uint8_t encPin)
    {
        this->inPin = encPin;
        pinMode(this->inPin, INPUT); // Set the pin used to measure the encoder to be an input
        freq.begin(this->inPin,FREQMEASUREMULTI_MARK_ONLY);
    }

    //updates rawAngle
    void RevEncoder::read(void) 
    {
        float sum = 0;
        int count = 0;
        if (freq.available()) {
            // average several reading together
            sum += freq.read();
            count +=1;
            if (count >= 1) // change 1 to avg reading across n readings
            {
                float frequency = freq.countToNanoseconds(sum / count) / 1000;
                ticks = (int) frequency % 1024;
                radians = (ticks/1024) * cmath.pi * 2;
                sum = 0;
                count = 0;
            }
        }
    }
}