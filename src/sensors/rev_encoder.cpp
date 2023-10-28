/**
 * @file rev_encoder.cpp
 * @brief The implementation of the Rev Encoder 
 * @date 2023-10-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <rev_encoder.hpp>

/**
 * @brief Construct a new Rev Encoder:: Rev Encoder object
 * 
 * @param encPin the pin number that the enoders signal pin is plugged into
 */
RevEncoder::RevEncoder(uint8_t encPin)
{
    this->inPin = encPin;
    pinMode(this->inPin, INPUT); // Set the pin used to measure the encoder to be an input
    Serial.begin(100000);
    freq.begin(this->inPin,FREQMEASUREMULTI_MARK_ONLY);
}

/**
 * @brief updates the current angle in ticks (1024) and radians
 */
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


