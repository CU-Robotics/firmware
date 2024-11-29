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
    float gyro_roll; // Roll axis
    float gyro_pitch; // pitch axis
    float accel_roll; // Roll axis
    float accel_pitch; // pitch axis
    float k_roll; // Roll angle filtered by KalmanFilter
    float k_pitch; // Pitch angle filtered by KalmanFilter 
    float temperature;
    float accel_scale;
};
class IMU_filter{
    private:
        ICM20649 _icm;

        IMUData _imu;

        Timer timer;

        // Simple filter
        float _alpha;
        // Kalman Filter
        float Q; // Process noise covariance 
        float R; // Measurement noise covariance
        std::array<std::array<float, 2>, 2> P; // State covariance matrix
        std::array<std::array<float, 2>, 2> K; // Kalman gain
    public:
        void init();
        IMUData* getdata();
        void read();
        void print();
        void calibrate_imu();
        void serial_data_for_plot();
};
#endif