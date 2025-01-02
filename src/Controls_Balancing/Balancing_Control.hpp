// #ifndef BALANCING_CONTROL_H
// #define BALANCING_CONTROL_H
// #include "../comms/rm_can.hpp"
// #include "../filters/pid_filter.hpp"
// #include "../utils/timing.hpp"
// /** Constants for Final OUTPUT*/ 
// #define WHEEL_UPPER_LIMIT 0.5                   //Need test
// #define WHEEL_LOWER_LIMIT -0.5                  //Need test
// #define F_WH_OUTPUT_LIMIT_NUM 5                 //Need test
// /** Constants for helping*/
// //for x[12] = [s, s_dot, phi, phi_dot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot, s_ddot, phi_ddot]
// #define XHELP_LENGTH 12                         
// #define XHELP_s 0                                       
// #define XHELP_s_dot 1
// #define XHELP_s_ddot 10
// #define XHELP_phi 2
// #define XHELP_phi_dot 3
// #define XHELP_phi_ddot 11
// #define XHELP_theta_ll 4
// #define XHELP_theta_ll_dot 5
// #define XHELP_theta_lr 6
// #define XHELP_theta_lr_dot 7
// #define XHELP_theta_b 8
// #define XHELP_theta_b_dot 9
// /** Constants for leg_controller*/
// #define m_b 1                                   //Need test
// #define m_l 1                                   //Need test
// #define R_l 1                                   //Need test
// #define eta_l 1                                 //Need test

// #define K1_P 1                                  //Need test                                    
// #define K1_I 1                                  //Need test 
// #define K1_D 1                                  //Need test                         
// #define K1_F 1                                  //Need test                         
// #define K2_P 1                                  //Need test                         
// #define K2_I 1                                  //Need test                         
// #define K2_D 1                                  //Need test                         
// #define K2_F 1                                  //Need test

// #define BOUND true                              // 1 to -1                      
// #define WARP true                               // 360 degree  

// //For gravity_ff
// #define G_CONSTANT 9.81                         //Need test

// /** Constants for locomotion_controller*/
// #define P_LOCO_ROW 4
// //For P_LOCO go to init() to define it


// /** Constants for mechanical*/
// #define NUM_MOTORS_Chassis 6
// #define T_LWL_OUTPUT_NUM 0
// #define T_LWR_OUTPUT_NUM 1
// #define T_JLF_OUTPUT_NUM 2
// #define T_JLB_OUTPUT_NUM 3
// #define T_JRF_OUTPUT_NUM 4
// #define T_JRB_OUTPUT_NUM 5
// /// @brief Manage all the balancing control 
// class BalancingControl{
//     private:
//     /** Helping Classes */
//         Timer timer;
//         /// @brief The PID for psi
//         PIDFilter pid1; 
//         /// @brief The PID for l
//         PIDFilter pid2; 

//     public:
//         float output[NUM_MOTORS_Chassis];
//         float p[6][P_LOCO_ROW][10];
//         /// @brief defalt constrctor 
//         BalancingControl();

//         void init();

//         /// @brief calculate the output and send them to the Can_bus
//         /// @param output for balancing contorl the data form is [T_lwl, T_jlf, T_jlb, T_lwr, T_jrf, T_jrb] 
//         /// @param ref array from ref {x_d[12] = ref[0-11], psi_d = ref[12], l_d = ref[13]}
//         /// @param obs array from observer {x[12] = obs[0-11], psi = obs[12], ll = obs[13], lr = obs[14], jl[2][2] = obs[15-18], jr[2][2] = obs[19-22], a_z = obs[23], ll_ddot = obs[24], lr_ddot = obs[25]}
//         void step(float output[NUM_MOTORS_Chassis], float ref[5][3], float obs[9][3]);
//         void printmotors(float output[NUM_MOTORS_Chassis]);
// };

// #endif