#ifndef BALANCING_CONTROL_H
#define BALANCING_CONTROL_H
#include "../comms/rm_can.hpp"
#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"

/** Constants for Final OUTPUT*/
#define WHEEL_UPPER_LIMIT 0.5
#define WHEEL_LOWER_LIMIT -0.5

/** Constants for helping*/
//for x[12] = [s, s_dot, phi, phi_dot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot, s_ddot, phi_ddot]
#define XHELP_LENGTH 12
#define XHELP_s 0
#define XHELP_s_dot 1
#define XHELP_s_ddot 10
#define XHELP_phi 2
#define XHELP_phi_dot 3
#define XHELP_phi_ddot 11
#define XHELP_theta_ll 4
#define XHELP_theta_ll_dot 5
#define XHELP_theta_lr 6
#define XHELP_theta_lr_dot 7
#define XHELP_theta_b 8
#define XHELP_theta_b_dot 9
/** Constants for leg_controller*/
#define m_b 
#define m_l 
#define R_l 
#define eta_l

#define K1_P 
#define K1_I 
#define K1_D 
#define K1_F 
#define K2_P 
#define K2_I 
#define K2_D 
#define K2_F 
#define BOUND true
#define WARP 
// MatrixMultiply 2x6 constant (If this doesn't need to change, need to simpfy the calculation) //ASK
//[a0][a1][a2][a3][a4][a5]
//[b0][b1][b2][b3][b4][b5]
#define MA0 1
#define MA1 1
#define MA2 1
#define MA3 0
#define MA4 1
#define MA5 0
#define MB0 -1
#define MB1 1
#define MB2 0 
#define MB3 1
#define MB4 0
#define MB5 1
//For gravity_ff
#define G_CONSTANT 9.81 
#define THE_C_IDK 0.2868 //ASK

/** Constants for locomotion_controller*/
#define P_LOCO_ROW 4
//For P_LOCO go to init() to define it


/** Constants for mechanical*/
#define NUM_MOTORS 6
#define T_LWL_OUTPUT_NUM 0
#define T_LWR_OUTPUT_NUM 1
#define T_JLF_OUTPUT_NUM 2
#define T_JLB_OUTPUT_NUM 3
#define T_JRF_OUTPUT_NUM 4
#define T_JRB_OUTPUT_NUM 5
/// @brief Manage all the balancing control 
class BalancingControl{
    private:
    /** Helping Classes */
        Timer timer;
        /// @brief The PID for psi
        PIDFilter pid1; 
        /// @brief The PID for l
        PIDFilter pid2; 

    public:
        float output[NUM_MOTORS];
        float p[6][P_LOCO_ROW][10];
        /// @brief defalt constrctor 
        BalancingControl();
        

        void init();

        /// @brief calculate the output and send them to the Can_bus
        /// @param output for balancing contorl the data form is [T_lwl, T_jlf, T_jlb, T_lwr, T_jrf, T_jrb] 
        void step(float output[NUM_MOTORS], float x_d[XHELP_LENGTH], float psi_d, float l_d, float x[XHELP_LENGTH], float psi, float ll, float lr, float jl[4], float jr[4], float a_z);
};

#endif