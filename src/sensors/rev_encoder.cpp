/**
 * @file rev_encoder.cpp
 * @brief The implementation of the Rev Encoder firmware 
 * @date 2023-10-14
 */

#include <rev_encoder.hpp>

RevEncoder::RevEncoder(uint8_t encPin, int baudrate)
{
    this->inPin = encPin;
    pinMode(this->inPin, INPUT); // Set the pin used to measure the encoder to be an input
    Serial.begin(baudrate);
    freq.begin(this->inPin, FREQMEASUREMULTI_MARK_ONLY);
}

void RevEncoder::read() 
{
    float sum = 0;
    int count = 0;  
    if (freq.available()) 
    {
        // average several reading together
        sum += freq.read();
        count++;
        if (count >= this->READ_SIZE) //averges the angle across READ_SIZE readings
        {
            float frequency = freq.countToNanoseconds(sum / count) / 1000;
            this->ticks = (int) frequency % 1024;
            this->radians = (this->ticks/1024) * M_PI * 2;
            sum = 0;
            count = 0;
        }
    }
}

float RevEncoder::getAngleTicks()
{
    return this->ticks;
}

float RevEncoder::getAngleRadians()
{
    return this->radians;
}