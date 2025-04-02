#ifndef IMU_FILTER_H
#define IMU_FILTER_H
#include "../utils/timing.hpp"
#include "../sensors/IMUSensor.hpp"
/// @brief the IMU filter class that filters the IMU data
class IMU_filter{
    private:
        /// @brief the timer for dt 
        Timer timer;
        /// @brief the time step
        float dt;
        /// @brief the quaternion for the filter
        float x[4];
        /// @brief process noise for model update
        float Q; 
        /// @brief process noise for accelerometer
        float R;
        /// @brief the prediction matrix
        std::array<std::array<float, 4>, 4> P;
        /// @brief the Kalman gain
        std::array<std::array<float, 4>, 4> K; // Kalman gain
        /// @brief the data structure that holds all the IMU data
        IMU_data filtered_data = {0};
        /// @brief the function for inverse 3x3 matrix
        /// @param mat input
        /// @param inv output
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