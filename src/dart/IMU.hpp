#ifndef IMU_H
#define IMU_H

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
  IMUData read_data();
  void print_data();
};

#endif
