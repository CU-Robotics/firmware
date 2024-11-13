#include <Adafruit_BMP280.h>
#include <BMI160Gen.h>
#include <CurieIMU.h>

#define Arduino_UNO
#define I2C

BMI160GenClass BMI160;
const int select_pin = 10;
Adafruit_BMP280 BMP280;

void setup() {
  Serial.begin(9600); // initialize Serial communication
  Serial.println("Programme de Test Capteur BMI+BMM");
  while (!Serial);    // wait for the serial port to open

  // initialize device
  BMI160.begin(select_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  BMI160.setGyroRange(250);
  BMI160.setAccelerometerRange(4);
}

void loop() {
  int axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, mxRaw, myRaw, mzRaw, tRaw;         // raw values
  float ax, ay, az, gx, gy, gz;

  // read raw gyro measurements from device
  BMI160.readMotionSensor9(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, mxRaw, myRaw, mzRaw, tRaw);
  
  // convert the raw gyro data to degrees/second
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);

  // convert the raw gyro data to degrees/second
  ax = convertRawAccel(axRaw);
  ay = convertRawAccel(ayRaw);
  az = convertRawAccel(azRaw);

  // display tab-separated gyro x/y/z values
  Serial.print("a:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.println();
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();
  Serial.print("m:\t");
  Serial.print(mxRaw);
  Serial.print("\t");
  Serial.print(myRaw);
  Serial.print("\t");
  Serial.print(mzRaw);
  Serial.println();
  Serial.print(tRaw);
  Serial.println();
  Serial.println();

  delay(500);
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

float convertRawAccel(int aRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float a = (aRaw * (4/32768.0));

  return a;
}

