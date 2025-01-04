#ifndef TEST_BALANCING_H
#define TEST_BALANCING_H

#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"
#include "../comms/can/can_manager.hpp"



//Constants for joint limit
#define limit_fl 5
#define limit_bl 
#define limit_fr 
#define limit_br 




//Constants for control
/** Constants for Final OUTPUT*/ 
#define WHEEL_UPPER_LIMIT 0.5                   //Need test
#define WHEEL_LOWER_LIMIT -0.5                  //Need test
#define F_WH_OUTPUT_LIMIT_NUM 5                 //Need test
/** Constants for leg_controller*/
#define m_b 1                                   //Need test
#define m_l 1                                   //Need test
#define R_l 1                                   //Need test
#define eta_l 1                                 //Need test

#define K1_P 1                                  //Need test                                    
#define K1_I 1                                  //Need test 
#define K1_D 1                                  //Need test                         
#define K1_F 1                                  //Need test                         
#define K2_P 1                                  //Need test                         
#define K2_I 1                                  //Need test                         
#define K2_D 1                                  //Need test                         
#define K2_F 1                                  //Need test

#define BOUND true                              // 1 to -1                      
#define WARP true                               // 360 degree                     

/** Constants for locomotion_controller*/
#define P_LOCO_ROW 4







struct balancing_sensor_data
{
    float angle_fl;
    float angle_bl;
    float angle_fr;
    float angle_br;

    float speed_fl;
    float speed_bl;
    float speed_fr;
    float speed_br;

    float speed_wl;
    float speed_wr;

    float imu_angle_pitch;
    float imu_angle_roll;
    float imu_accel_x;
    float imu_accel_y;
    float imu_accel_z;
    float gyro_pitch;
    float gyro_roll;
    float gyro_yew;
};





class balancing_test{
    private:
        Timer timer; 
        /// @brief The PID for psi
        PIDFilter pid1; 
        /// @brief The PID for l
        PIDFilter pid2; 


        float p[6][P_LOCO_ROW][10];


        balancing_sensor_data _data;
        float _torque_fl;
        float _torque_bl;
        float _torque_fr;
        float _torque_br;
        float _torque_wl;
        float _torque_wr;

    public:
        /// @brief setting all constant array
        void init();
        /// @brief Set private data the data from main
        /// @param balancing_sensor_data struct which stores all sensor data needed
        void set_data(balancing_sensor_data);
        /// @brief write all the torque data with a protection limit
        void limit_write();

        void observer();

        void control();

        void control_position();

        void step();



};




#endif 