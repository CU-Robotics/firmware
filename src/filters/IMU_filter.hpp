#ifndef IMU_FILTER_H
#define IMU_FILTER_H
#include "../utils/timing.hpp"
#include "../sensors/IMUSensor.hpp"

class IMU_filter{
    private:
        Timer timer;
        float dt;

        float x[4];

        float Q; //process noise for angle
        float R; //process noise for accelerometer
        float chi_square; //Chi-squared test
        std::array<std::array<float, 4>, 4> P; // estimate covariance
        std::array<std::array<float, 4>, 4> K; // Kalman gain
        IMU_data filtered_data;
        void inverse3x3(float mat[3][3], float inv[3][3]); 

        

    public:
        /// @brief Initalize everything including filter constant
        void init_EKF_6axis(IMU_data);
        
        /// @brief Do one step of the EKF filter
        /// @param  IMU_data is the IMU data structure that holds all the IMU data
        /// @return int 0 if successful
        int step_EKF_6axis(IMU_data); 

        /// @brief Print out data for debugging
        void print();
        /// @brief Print out data for a Python 3D visulize function
        void serial_data_for_plot();
        /// @brief get the filtered data
        /// @return IMU data structure
        IMU_data* get_filter_data();
};
#endif