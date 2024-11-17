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
    float alpha_X; // Angular acceleration left and right
    float alpha_Y; // front and back
    float alpha_Z; // Rotation 
    float angle_X; // right(+) and left(-) --flat is 0 
    float angle_Y; // front(+) and back(-) --flat is 0
    float temperature;

    float accel_scale;
};
class IMU_filter{
    private:
        ICM20649 _icm;

        IMUData _imu;

        Timer timer;
    public:
        void init();
        IMUData* getdata();
        void read();
        void print();
        void calibrate_imu();
};
#endif