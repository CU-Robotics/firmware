#ifndef BALANCINGIMU_FILTER_H
#define BALANCINGIMU_FILTER_H
#include "./sensors/ICM20649.hpp"
struct IMUData{
    float accel_X;
    float accel_Y;
    float accel_Z; 
    float gyro_X;
    float gyro_Y;
    float gyro_Z;
    float temperature;
};
class IMU_filter{
    private:
        ICM20649 icm;
    public:
        IMUData imu;
        void read();
};
#endif