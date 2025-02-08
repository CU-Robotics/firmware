#ifndef IMU_FILTER_H
#define IMU_FILTER_H
#include "../utils/timing.hpp"

class IMU_filter{
    private:
        Timer timer;

        
    public:
        /// @brief Initalize everything including filter constant
        void init_EKF_6axis();
        /// @brief Print out data for debugging
        void print();
        /// @brief Print out data for a Python 3D visulize function
        void serial_data_for_plot();
};
#endif