#ifndef IMU_FILTER_H
#define IMU_FILTER_H
#include "../utils/timing.hpp"
#include "../sensors/IMUSensor.hpp"

class IMU_filter{
    private:
        Timer timer;

        float x[4];
    public:
        /// @brief Initalize everything including filter constant
        void init_EKF_6axis(IMU_data);
        
        void step_EKF_6axis(IMU_data);
        /// @brief Print out data for debugging
        void print();
        /// @brief Print out data for a Python 3D visulize function
        void serial_data_for_plot();

        
};
#endif