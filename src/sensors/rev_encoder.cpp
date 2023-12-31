#include "rev_encoder.hpp"

RevEncoder::RevEncoder(uint8_t encoder_pin, int baudrate) 
{
  this->in_pin = encoder_pin;
  pinMode(this->in_pin, INPUT);  // Set the pin used to measure the encoder to be an input
  Serial.begin(baudrate);
  freq.begin(this->in_pin, FREQMEASUREMULTI_MARK_ONLY);
}

void RevEncoder::read() 
{
  while (this->freq.available() > 1) 
  {
		this->freq.read();
	}
  if (freq.available()) 
  {
      int frequency = round(this->freq.countToNanoseconds(this->freq.read()) / 1000);
      this->ticks = frequency % 1024;
      this->radians =  (((float) this->ticks) / 1024.0) * M_PI * 2;
  }
}

float RevEncoder::get_angle_ticks() 
{
  return this->ticks;
}

float RevEncoder::get_angle_radians()
{
  return this->radians;
}
