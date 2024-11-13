// IMU.h

#pragma once

struct IMUData {
  float roll;
  float pitch;
  float yaw;
  float accelX;
  float accelY;
  float accelZ;
};

class IMU {
public:
  IMU();
  void init();
  IMUData readData();
};
