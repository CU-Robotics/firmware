/*
uNavAHRS_MPU9250.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "uNavAHRS.h"
// a uNavAHRS object
uNavAHRS Filter;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float ypr[4], ypr_kf[3];
float q[4];
#define G 9.80665;

// timers to measure performance
unsigned long tstart, tstop;
elapsedMillis sincePrint;
double roll, pitch, heading;
int MagConst[3];

#include <BMI160Gen.h>
#include "bmm150.h"
#include "bmm150_defs.h"

BMM150 bmm = BMM150();


uint8_t drdy = 0;
uint8_t bmiDRDY = 0;
uint8_t bmm150_drdy = 0;

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 2;
const int bmm150_interrupt_pin = 5;


void bmi160_intr(void)
{
  bmiDRDY = 1;
}

void bmm_drdy(){
  bmm150_drdy = 1;
}

int gRange, aRange;
uint8_t initDataRdy;


void setup() {
// initialize device
  Serial.begin(9600); // initialize Serial communication
  while(!Serial && millis()<2000) {}
  
  // initialize device
  Serial.println("Initializing IMU device...");

  // verify connection
  BMI160.begin(BMI160GenClass::I2C_MODE, bmi160_i2c_addr, bmi160_interrupt_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  //BMI160.setGyroRate(25);
  BMI160.setAccelerometerRange(2);
  BMI160.setAccelRate(BMI160_ACCEL_RATE_100HZ);
  //BMI160.setAccelDLPFMode(BMI160_DLPF_MODE_NORM);
  BMI160.setGyroRange(250);
  BMI160.setGyroRate(BMI160_GYRO_RATE_100HZ);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    BMI160.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(BMI160.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(BMI160.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(BMI160.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(BMI160.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(BMI160.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(BMI160.getGyroOffset(Z_AXIS));

    BMI160.attachInterrupt(bmi160_intr);
    BMI160.setIntDataReadyEnabled(true);
    
  if(bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while(1);
  } else {
    Serial.println("Initialize done!");
  }

  //set BMM150 ODR
  bmm.set_mag_rate(BMM150_DATA_RATE_25HZ);
  //Enable BMM150 interrupt
  bmm.set_DRDY_bit(0);

  //pinMode(bmm150_interrupt_pin, INPUT);
  //attachInterrupt(bmm150_interrupt_pin, bmm_drdy, RISING);

  MagConst[0] = -1;
  MagConst[1] = 28;
  MagConst[2] = -46;
  
  gRange = BMI160.getGyroRange();
  aRange = BMI160.getAccelerometerRange();


  Filter.setAccelCovariance(0.025f); // also try 0.025f
  Filter.setHeadingCovariance(0.0125f); // also try 0.025f

}

void loop() {
  int gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw, tRaw;         // raw gyro values
  int mxRaw, myRaw, mzRaw;         // raw mag values
  
  if(bmiDRDY == 1){
    tstart = micros();
	  bmiDRDY = 0;
    
	  // read raw gyro measurements from device
      BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

	  // convert the raw gyro data to degrees/second
	  gx = convertRaw(gxRaw, gRange)*PI/180;
	  gy = convertRaw(gyRaw, gRange)*PI/180;
	  gz = convertRaw(gzRaw, gRange)*PI/180;
    
	  ax = convertRaw(axRaw, aRange)*G;
	  ay = convertRaw(ayRaw, aRange)*G;
	  az = convertRaw(azRaw, aRange)*G;

      //bmm150_mag_data value;
      //if(bmm150_drdy == 1){
        bmm.read_mag_data();
        mx = (bmm.raw_mag_data.raw_datax-MagConst[0]);
        my = (bmm.raw_mag_data.raw_datay-MagConst[1]);
        mz = (bmm.raw_mag_data.raw_dataz-MagConst[2]); 
        bmm150_drdy == 0;  
      //}
/*
  Serial.print("g:\t");
  Serial.print(gx); Serial.print("\t"); Serial.print(gy);
  Serial.print("\t");  Serial.print(gz);
  Serial.print("\t a:\t");
  Serial.print(ax); Serial.print("\t"); Serial.print(ay);
  Serial.print("\t"); Serial.print(az);
  Serial.print("\t m:\t"); Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t"); Serial.print(mz);
  Serial.println();
*/	  

    Filter.update(gy, gx, -gz, ay, ax, -az, mx, my, mz);
    tstop = micros();
	
/*    Serial.print(Filter.getPitch_rad()*180.0f/PI);
    Serial.print("\t");
    Serial.print(Filter.getRoll_rad()*180.0f/PI);
    Serial.print("\t");
    Serial.println(Filter.getYaw_rad()*180.0f/PI);
    Serial.print("\t");
    Serial.println(Filter.getHeading_rad()*180.0f/PI);
*/
    if(sincePrint >100){
      sincePrint = 0;
      float yaw = Filter.getYaw_rad()*180.0f/PI;
      float pitch = Filter.getPitch_rad()*180.0f/PI;
      float roll = Filter.getRoll_rad()*180.0f/PI;
      //float heading = Filter.getHeading_rad()*180.0f/PI;
    
      char yaw_text[30];
      char pitch_text[30];
      char roll_text[30];
      //char heading_text[30];
    
      dtostrf(yaw, 10, 10, yaw_text);
      dtostrf(pitch, 10, 10, pitch_text);
      dtostrf(roll, 10, 10, roll_text);
      //dtostrf(heading, 10, 10, heading_text);
    
      char text[93];
      snprintf(text, 93, "%s,%s,%s", roll_text, pitch_text, yaw_text);
      Serial.println(text);
    }
  }
}

float convertRaw(int16_t raw, float range_abs)
{
    float slope;
    float val;

    /* Input range will be -32768 to 32767
     * Output range must be -range_abs to range_abs */
    val = (float)raw;
    slope = (range_abs * 2.0f) / BMI160_SENSOR_RANGE;
    return -(range_abs) + slope * (val + BMI160_SENSOR_LOW);
}

int getAccelerometerRange()
{
    int range;

    switch (BMI160.getFullScaleAccelRange()) {
        case BMI160_ACCEL_RANGE_2G:
            range = 2;
            break;

        case BMI160_ACCEL_RANGE_4G:
            range = 4;
            break;

        case BMI160_ACCEL_RANGE_8G:
            range = 8;
            break;

        case BMI160_ACCEL_RANGE_16G:
        default:
            range = 16;
            break;
    }

    return range;
}

int getGyroRange()
{
    int range;

    switch (BMI160.getFullScaleGyroRange()) {
        case BMI160_GYRO_RANGE_2000:
            range = 2000;
            break;

        case BMI160_GYRO_RANGE_1000:
            range = 1000;
            break;

        case BMI160_GYRO_RANGE_500:
            range = 500;
            break;

        case BMI160_GYRO_RANGE_250:
            range = 250;
            break;

        case BMI160_GYRO_RANGE_125:
        default:
            range = 125;
            break;
    }

    return range;
}
