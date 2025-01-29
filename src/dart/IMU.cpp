#include "IMU.hpp"
#include "Adafruit_ICM20649.h"
#include "Adafruit_Sensor.h"
#include <Wire.h>

#define SPI_CS_PIN 10

Adafruit_ICM20649 icm;

IMU::IMU(){};

void IMU::init() {
  Serial.println("ICM Init Started");

  if (!icm.begin_SPI(10, &SPI)) {
    Serial.println("ICM-20649 not detected!");
    while (1)
      ;
  }
  Serial.println("ICM-20649 initialized with SPI!");

  icm.setAccelRange(ICM20649_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS);
  icm.setAccelRateDivisor(0);
  icm.setGyroRateDivisor(0);
}

IMUData IMU::read_data() {
  IMUData data;

  sensors_event_t accel, gyro, temp;

  icm.getEvent(&accel, &gyro, &temp);
  data.roll = gyro.gyro.x;
  data.pitch = gyro.gyro.y;
  data.yaw = gyro.gyro.z;

  data.accelX = accel.acceleration.x;
  data.accelY = accel.acceleration.y;
  data.accelZ = accel.acceleration.z;

  return data;
}

void IMU::print_data() {
  sensors_event_t accel, gyro, temp;

  icm.getEvent(&accel, &gyro, &temp);

  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(accel.acceleration.y);
  Serial.print(", Z: ");
  Serial.println(accel.acceleration.z);

  Serial.print("Gyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(", Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print(", Z: ");
  Serial.println(gyro.gyro.z);

  Serial.print("Temp: ");
  Serial.println(temp.temperature);
}
