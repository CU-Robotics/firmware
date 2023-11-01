/**
 * @file rev_encoder.cpp
 * @brief The implementation of the Rev Encoder 
 * @date 2023-10-31
 */
#include <rev_encoder.hpp>

RevEncoder::RevEncoder(uint8_t encPin, int baudrate) 
{
  this->inPin = encPin;
  this->sum = 0;
  this->count = 0;
  pinMode(this->inPin, INPUT);  // Set the pin used to measure the encoder to be an input
  Serial.begin(baudrate);
  freq.begin(this->inPin, FREQMEASUREMULTI_MARK_ONLY);
  
}

void RevEncoder::read() 
{
  if (freq.available()) 
  {
    // average several reading together
    this->sum += this->freq.read();
    this->count++;
    if (this->count >= this->READ_SIZE)  //averges the angle across n readings
    {
      float frequency = freq.countToNanoseconds(sum / count) / 1000;
      this->ticks = (int) frequency % 1024;
      this->radians = (this->ticks / 1024) * M_PI * 2;
      this->sum = 0;
      this->count = 0;
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
