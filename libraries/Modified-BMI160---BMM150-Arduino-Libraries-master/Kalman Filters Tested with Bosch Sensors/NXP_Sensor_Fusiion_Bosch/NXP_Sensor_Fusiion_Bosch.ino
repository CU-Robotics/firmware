
#include <NXPMotionSense.h>

#include <elapsedMillis.h>

#include "FilteringScheme.h"
KalmanFilter kFilters[4];
int k_index = 3;


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

// a Sensor Fusion object
NXPSensorFusion filter;

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

void setup (){
	
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
  bmm.set_DRDY_bit(1);

  pinMode(bmm150_interrupt_pin, INPUT);
  attachInterrupt(bmm150_interrupt_pin, bmm_drdy, RISING);

  MagConst[0] = 31;
  MagConst[1] = -8;
  MagConst[2] = -42;
  
  gRange = BMI160.getGyroRange();
  aRange = BMI160.getAccelerometerRange();

  filter.begin(100); // 100 measurements per second

  delay(4000);  
  
}

void loop(){
  int gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw, tRaw;         // raw gyro values
  int mxRaw, myRaw, mzRaw;         // raw gyro values

	 if(bmiDRDY == 1){
	  // read raw gyro measurements from device
    BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

	  // convert the raw gyro data to degrees/second
	  gx = convertRaw(gxRaw, gRange);
	  gy = convertRaw(gyRaw, gRange);
	  gz = convertRaw(gzRaw, gRange);
    
	  ax = convertRaw(axRaw, aRange)*G;
	  ay = convertRaw(ayRaw, aRange)*G;
	  az = convertRaw(azRaw, aRange)*G;

      //bmm150_mag_data value;
      //if(bmm150_drdy == 1){
        bmm.read_mag_data();
        mx = (bmm.raw_mag_data.raw_datay-MagConst[0]);
        my = (bmm.raw_mag_data.raw_datax-MagConst[1]);
        mz = -(bmm.raw_mag_data.raw_dataz-MagConst[2]); 
        //bmm150_drdy == 0;  
      //}

     /*run KalmanFilter*/
     filter.update(gx, gy, gz, ax, ay, az, mx, my, mz); 
    
     if(sincePrint > 100){
      /*print the resultant quaternion to serial terminal*/
      //Serial.printf("Q: %f %f %f %f \r\n",data_matrix_X_prev[0],data_matrix_X_prev[1],data_matrix_X_prev[2],data_matrix_X_prev[3]);      
      //Determine orientation based on Quaternion
      //filter.getQuaternion(q);
      //getYawPitchRollDeg();
      ypr[1] = filter.getPitch();
      ypr[2] = filter.getRoll();
      ypr[0] = filter.getYaw();

      masterSerialPrint();
      sincePrint = 0;
     }
    bmiDRDY = 0;
	}
}


void getYawPitchRollDeg() {

  float gx, gy, gz; // estimated gravity direction
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = (180/PI) * atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = -(180/PI) * atan(gx / sqrt(gy*gy + gz*gz));
  ypr[2] = -(180/PI) * atan(gy / sqrt(gx*gx + gz*gz));

  //Serial.print(ypr[0]); Serial.print(","); 
  //Serial.print(ypr[1]); Serial.print(",");
  //Serial.print(ypr[2]);
  Serial.println();
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
