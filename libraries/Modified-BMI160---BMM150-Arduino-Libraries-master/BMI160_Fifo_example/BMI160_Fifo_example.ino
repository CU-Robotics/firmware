
#include <BMI160Gen.h>

  int fifo_cnt;
  uint8_t data[1024];
  int16_t ax, ay, az;         // accelerometer values
  int16_t gx, gy, gz;         // gyrometer values

void bmi160_intr(void)
{
  fifo_cnt = BMI160.getFIFOCount();
  fifo_cnt += 4;
  BMI160.getFIFOBytes(data, fifo_cnt);
  decode();
  //show_fifo_frames();
}

void decode(){
  gx = (((int16_t)data[13])  << 8) | data[12];
  ax = (((int16_t)data[19])  << 8) | data[18];

  gy = (((int16_t)data[15])  << 8) | data[14];
  ay = (((int16_t)data[21])  << 8) | data[20];
  
  gz = (((int16_t)data[17])  << 8) | data[16];
  az = (((int16_t)data[23])  << 8) | data[22];
  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax);  Serial.print("\t"); Serial.print(ay); Serial.print("\t");
  Serial.print(az);  Serial.print("\t");
  Serial.print(gx);  Serial.print("\t"); Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
}

void show_fifo_frames(){
  //Serial.println(fifo_cnt);
  for(int i = 0; i < fifo_cnt; i++){
    Serial.print(data[i]); Serial.print(", ");
  }
  Serial.println("----------------------");
}


void setup() {
// initialize device
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println("Initializing IMU device...");

  // verify connection
  //BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
  BMI160.begin(BMI160GenClass::I2C_MODE, 0x69, 2);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  //BMI160.setGyroRate(25);
  BMI160.setAccelerometerRange(2);
  BMI160.setAccelRate(BMI160_ACCEL_RATE_1600HZ);
  //BMI160.setAccelDLPFMode(BMI160_DLPF_MODE_NORM);
  BMI160.setGyroRange(250);
  BMI160.setGyroRate(BMI160_GYRO_RATE_1600HZ);

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
  
  BMI160.setFIFOHeaderModeEnabled(false);   // Use Headerless Mode
  BMI160.setAccelFIFOEnabled(true);         // Enable the accelerometer FIFO
  BMI160.setGyroFIFOEnabled(true);          // Enable the gyro FIFO

  #define BMI160_FIFO_TIME_EN_BIT 1           // Returns a sensor time after last valid frame
  BMI160.reg_write_bits(BMI160_RA_FIFO_CONFIG_1, 0x1, BMI160_FIFO_TIME_EN_BIT, 1);

  BMI160.attachInterrupt(bmi160_intr);      // Attach interrupt for when FIFO data greater than
                                              // watermark level
  //BMI160.reg_write(BMI160_RA_FIFO_CONFIG_0, 0b00000101); // set FIFO water mark level
  BMI160.reg_write(BMI160_RA_FIFO_CONFIG_0, 0b00001010); // set FIFO water mark level
  #define BMI160_FWM_INT_BIT 6                // Sets watermark interrupt
  BMI160.reg_write_bits(BMI160_RA_INT_EN_1, 0x1, BMI160_FWM_INT_BIT, 1);
  BMI160.resetFIFO();

  // Some debug info to verify register writes
  //Serial.println(BMI160.reg_read(BMI160_RA_FIFO_CONFIG_0), BIN);
  //Serial.println(BMI160.reg_read(BMI160_RA_FIFO_CONFIG_1), BIN);
  //Serial.println(CurieIMU.reg_read(BMI160_RA_INT_EN_1),BIN);
}

void loop() {

  delay(1);
}
