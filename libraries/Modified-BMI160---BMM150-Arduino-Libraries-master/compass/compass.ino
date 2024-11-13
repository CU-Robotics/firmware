/**
 * This example 
*/

#include <Arduino.h>
#include <Wire.h>
// libraries
#include "bmm150.h"
#include "bmm150_defs.h"

BMM150 bmm = BMM150();

uint8_t drdy = 0;

void isr_drdy(){
  drdy = 1;
  //Serial.println("tripped");
}

void setup()
{
  Serial.begin(9600);
  delay(5000);

  if(bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while(1);
  } else {
    Serial.println("Initialize done!");
  }

  pinMode(5, INPUT);
  attachInterrupt(5, isr_drdy, RISING);
}

void loop()
{
  //Serial.println(drdy);
 
    bmm150_mag_data value;
    bmm.read_mag_data();

  if(drdy == 1){ 
    value.x = bmm.raw_mag_data.raw_datax;
    value.y = bmm.raw_mag_data.raw_datay;
    value.z = bmm.raw_mag_data.raw_dataz;
  
    float xyHeading = atan2(value.x, value.y);
    float zxHeading = atan2(value.z, value.x);
    float heading = xyHeading;
  
    if(heading < 0)
      heading += 2*PI;
    if(heading > 2*PI)
      heading -= 2*PI;
    float headingDegrees = heading * 180/M_PI; 
    float xyHeadingDegrees = xyHeading * 180 / M_PI;
    float zxHeadingDegrees = zxHeading * 180 / M_PI;
  
    Serial.print("Heading: ");
    Serial.println(headingDegrees);

    //delay(10); 
  }
  drdy = 0;
}
