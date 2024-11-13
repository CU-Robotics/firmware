
#include <arm_math.h>

#include <elapsedMillis.h>
#include "YAKF.h"

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

void setup (){
	
// initialize device
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

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


   
  /* USER CODE BEGIN 2 */
  data_matrix_P_prev[0] = 1;
  data_matrix_P_prev[1] = 0;
  data_matrix_P_prev[2] = 0;
  data_matrix_P_prev[3] = 0;
  data_matrix_P_prev[4] = 0;
  data_matrix_P_prev[5] = 1;
  data_matrix_P_prev[6] = 0;
  data_matrix_P_prev[7] = 0;
  data_matrix_P_prev[8] = 0;
  data_matrix_P_prev[9] = 0;
  data_matrix_P_prev[10] = 1;
  data_matrix_P_prev[11] = 0;
  data_matrix_P_prev[12] = 0;
  data_matrix_P_prev[13] = 0;
  data_matrix_P_prev[14] = 0;
  data_matrix_P_prev[15] = 1;
  
  data_matrix_X_prev[0] = 1;
  data_matrix_X_prev[1] = 0;
  data_matrix_X_prev[2] = 0;
  data_matrix_X_prev[3] = 0;
  data_matrix_X[0] = 1;
  data_matrix_X[1] = 0;
  data_matrix_X[2] = 0;
  data_matrix_X[3] = 0;
  
  /* KF step 0-> initialize*/
  initMatrix();
  
  delay(4000);  
  
}

void loop(){
  int gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw, tRaw;         // raw gyro values
  int mxRaw, myRaw, mzRaw;         // raw gyro values

  if (!initDataRdy){ 
	if(bmiDRDY == 1){
	 // read raw gyro measurements from device
	 BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);
    
	  // convert the raw gyro data to degrees/second
	  Gxyz[0] = convertRaw(gxRaw, gRange);
	  Gxyz[1] = convertRaw(gyRaw, gRange);
	  Gxyz[2] = convertRaw(gzRaw, gRange);
    
	  Axyz[1] = convertRaw(axRaw, aRange)*G;
	  Axyz[1] = convertRaw(ayRaw, aRange)*G;
	  Axyz[2] = convertRaw(azRaw, aRange)*G;
/*
      bmm150_mag_data value;
      bmm.read_mag_data();
      Mxyz[0] = (bmm.raw_mag_data.raw_datay);
      Mxyz[1] = (bmm.raw_mag_data.raw_datax);
      Mxyz[2] = -(bmm.raw_mag_data.raw_dataz);      
*/ 
      if(counter<100){
        for (int i=0;i<3;i++){
          //sMag[i] = sMag[i] + Mxyz[i];
          sAcc[i] = sAcc[i] + Axyz[i];
          sGyro[i] = sGyro[i] + Gxyz[i];           
        }
        counter = counter+1;
      }    
      if (counter==100){
        for (int i=0;i<3;i++){
          //MagConst[i] = sMag[i]/100;
          gravity[i] = sAcc[i]/100;
          gyroBias[i] = sGyro[i]/100;
        }
        initDataRdy = 1;
      }
	  bmiDRDY = 0;
	}
  } else {
	 if(bmiDRDY == 1){
	  // read raw gyro measurements from device
    BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

	  // convert the raw gyro data to degrees/second
	  Gxyz[0] = convertRaw(gxRaw, gRange);
	  Gxyz[1] = convertRaw(gyRaw, gRange);
	  Gxyz[2] = convertRaw(gzRaw, gRange);
    
	  Axyz[1] = convertRaw(axRaw, aRange)*G;
	  Axyz[1] = convertRaw(ayRaw, aRange)*G;
	  Axyz[2] = convertRaw(azRaw, aRange)*G;

      //bmm150_mag_data value;
      //if(bmm150_drdy == 1){
        bmm.read_mag_data();
        Mxyz[0] = (bmm.raw_mag_data.raw_datay);
        Mxyz[1] = (bmm.raw_mag_data.raw_datax);
        Mxyz[2] = -(bmm.raw_mag_data.raw_dataz); 
        //bmm150_drdy == 0;  
      //}
      
     //High Pass Filter -> remove all values that are less than 0.05dps.
     for (int i=0;i<3;i++){
       if(Gxyz[i]<0.05){
         Gxyz[i]=0;
       }
     }
      
     /*run KalmanFilter*/
     runKalmanFilter();
    
     if(sincePrint > 100){
      /*print the resultant quaternion to serial terminal*/
      //Serial.printf("Q: %f %f %f %f \r\n",data_matrix_X_prev[0],data_matrix_X_prev[1],data_matrix_X_prev[2],data_matrix_X_prev[3]);      
      q[0] = data_matrix_X_prev[0];
      q[1] = data_matrix_X_prev[1];
      q[2] = data_matrix_X_prev[2];
      q[3] = data_matrix_X_prev[3];
      //q[0] = kFilters[0].measureRSSI(data_matrix_X_prev[0]);
      //q[1] = kFilters[1].measureRSSI(data_matrix_X_prev[1]);
      //q[2] = kFilters[2].measureRSSI(data_matrix_X_prev[2]);
      //q[3] = kFilters[2].measureRSSI(data_matrix_X_prev[3]);
      getYawPitchRollDeg();

      masterSerialPrint();
      sincePrint = 0;
     }
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
