/** @author I am going to make my own github to explain the whole math for IMU 
 * Here will be the link for it. Just check and you guys should understand it quickly
 * 
 * 
 * 
 * 
 * */ 



#ifndef IMU_FILTER_H
#define IMU_FILTER_H
#include "../sensors/ICM20649.hpp"
#include "../utils/timing.hpp"

#define ACCEL_SENSITIVITY 
#define CALIBRATION_NUM 100000
struct IMUData{
    float accel_X; // acceleration of X axis --unit (m/s^2)
    float accel_Y; // acceleration of Y axis --unit (m/s^2)
    float accel_Z; // acceleration of Z axis --unit (m/s^2)
    float world_accel_X; // world acceleration of X axis --unit (m/s^2)
    float world_accel_Y; // world acceleration of Y axis --unit (m/s^2)
    float world_accel_Z; // world acceleration of Z axis --unit (m/s^2)
    float alpha_roll; // Roll axis rotation velocity --unit (rad/s) 
    float alpha_pitch; // pitch axis rotation velocity --unit (rad/s) 
    float world_alpha_roll;
    float world_alpha_pitch;
    float world_alpha_yaw;
    float alpha_yaw; // Yaw axis rotation velocity --unit (rad/s) 
    float gyro_roll; // Roll angle by gyroscope with a simple filter --unit (rad) 
    float gyro_pitch; // Pitch angle by gyroscope with a simple filter --unit (rad) 
    float accel_roll; // Roll angle by accelration --unit (rad) 
    float accel_pitch; // Pitch angle by accelration --unit (rad) 
    float k_roll; // Roll angle filtered by KalmanFilter --unit (rad)   --- This is the result we want
    float k_pitch; // Pitch angle filtered by KalmanFilter --unit (rad) --- This is the result we want
    float temperature; // --unit (Celsius) 
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
        /// @brief Initalize everything including filter constant
        void init();
        /// @brief calibrate errors and scale for IMU
        void calibrate_imu();
        /// @brief This is a function to get IMUData return
        /// @return An IMUData filled with almost everything from the IMU sensor
        IMUData getdata();
        /// @brief This is the read function (Also step function) without Yaw and works for +/- 90 degree  
        void read();
        /// @brief Print out data for debugging
        void print();
        /// @brief Print out data for a Python 3D visulize function
        void serial_data_for_plot();
};
#endif