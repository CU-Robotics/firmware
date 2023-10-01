#include <FreqMeasureMulti.h>
#include <cmath>

float ticks, radians;
uint8_t inPin;
FreqMeasureMulti freq;

void setup()
{
  Serial.begin(9600);
  inPin = 4;
  pinMode(inPin, INPUT);
  freq.begin(inPin, FREQMEASUREMULTI_MARK_ONLY);

}

void loop () 
{
  float sum = 0;
  int count = 0;
  if (freq.available()) 
  {
      // average several reading together
      sum += freq.read();
      count +=1;
      if (count >= 1) // change 1 to avg reading across n readings
      {
          float frequency = freq.countToNanoseconds(sum / count) / 1000;
          ticks = (int) frequency % 1024;
          radians = (ticks/1024) * M_PI * 2;
          sum = 0;
          count = 0;
      }
  }
  Serial.println(radians);

}
