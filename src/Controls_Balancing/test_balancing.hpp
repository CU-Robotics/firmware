#ifndef TEST_BALANCING_H
#define TEST_BALANCING_H

#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"
#include "../comms/can/can_manager.hpp"
//Constants for control
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

/** Constants for locomotion_controller*/
#define P_LOCO_ROW 4

/**observer constants */
#define l_a 0   // test
#define l_u 0   // test
#define l_l 0   // test
#define R_w 0   // test

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


struct write_data
{
    float torque_fl;
    float torque_bl;
    float torque_fr;
    float torque_br;
    float torque_wl;
    float torque_wr;
};
struct observer_data
{
    float pitch_dot;
    float yaw_dot;
    float yaw_ddot;
    float b_speed;
    float b_accel;
    float ll;
    float lr;
    float ll_ddot;
    float lr_ddot;
    float theta_ll;
    float theta_lr;
    float theta_ll_dot;
    float theta_lr_dot;
    float jl[2][2];
    float jr[2][2];

    float pitch_old;
    float yaw_dot_old;
    float theta_ll_old;
    float theta_lr_old;
    float b_speed_old;
    float ll_old;
    float ll_dot_old;
    float lr_old;
    float lr_dot_old;
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
        write_data _write;
        observer_data o_data;

    public:
        /// @brief setting all constant array
        void init();
        /// @brief Set private data the data from main
        /// @param balancing_sensor_data struct which stores all sensor data needed
        void set_data(balancing_sensor_data);
        /// @brief write all the torque data with a protection limit
        void limit_write();

        void test_write();
        
        void observer();

        void control();

        void control_position();

        void step();

        write_data getwrite();

        void printdata();

};




#endif 