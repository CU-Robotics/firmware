#ifndef IMU_FILTER_H
#define IMU_FILTER_H
#include "../sensors/ICM20649.hpp"
#include "../utils/timing.hpp"

#define ACCEL_SENSITIVITY 
#define CALIBRATION_NUM 100000
struct IMUData{
    float accel_X;
    float accel_Y;
    float accel_Z; 
    float alpha_X; // Roll axis
    float alpha_Y; // pitch axis
    float alpha_Z; // Yaw axis 
    float angle_X; // Roll axis
    float angle_Y; // pitch axis
    float acclangle_X; // Roll axis
    float acclangle_Y; // pitch axis 
    float temperature;
    float accel_scale;
};
class IMU_filter{
    private:
        ICM20649 _icm;

        IMUData _imu;

        float _alpha;

        Timer timer;
    public:
        void init();
        IMUData* getdata();
        void read();
        void print();
        void calibrate_imu();
};
#endif