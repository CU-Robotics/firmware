
#include <BMI160Gen.h>
#include "bmm150.h"
#include "bmm150_defs.h"

BMM150 bmm = BMM150();

uint8_t drdy = 0;
uint8_t bmiDRDY = 0;

void isr_drdy(){
  drdy = 1;
  //Serial.println("tripped");
}

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 2;
const int bmm150_interrupt_pin = 5;

void bmi160_intr(void)
{
  bmiDRDY = 1;

}

int gRange, aRange;

void setup(){
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

  gRange = BMI160.getGyroRange();
  aRange = BMI160.getAccelerometerRange();

  pinMode(bmm150_interrupt_pin, INPUT);
  attachInterrupt(bmm150_interrupt_pin, isr_drdy, RISING);
    
}

void loop(){
  int gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw;         // raw gyro values
  float gx, gy, gz, ax, ay, az;

 if(bmiDRDY == 1){

  // read raw gyro measurements from device
  BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRaw(gxRaw, gRange);
  gy = convertRaw(gyRaw, gRange);
  gz = convertRaw(gzRaw, gRange);
  ax = convertRaw(axRaw, aRange);
  ay = convertRaw(ayRaw, aRange);
  az = convertRaw(azRaw, aRange);
  
  // display tab-separated gyro x/y/z values
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.print("\t a:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.println();

  bmiDRDY = 0;
 }

    bmm150_mag_data value;
    bmm.read_mag_data();

  if(drdy == 1){ 
    value.x = bmm.raw_mag_data.raw_datax;
    value.y = bmm.raw_mag_data.raw_datay;
    value.z = bmm.raw_mag_data.raw_dataz;
 /* 
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
*/
  Serial.print("m:\t");
  Serial.print(value.x);
  Serial.print("\t");
  Serial.print(value.y);
  Serial.print("\t");
  Serial.print(value.y);
  Serial.println(); Serial.println();
    //delay(10); 
  }
  drdy = 0;
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
