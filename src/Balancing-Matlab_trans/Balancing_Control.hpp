#ifndef BALANCING_CONTROL_H
#define BALANCING_CONTROL_H
#include "../comms/rm_can.hpp"
#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"

/** Constants for helping*/
//for x[12] = [s, s_dot, s_ddot, phi, phi_dot, phi_ddot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot]
#define XHELP_s 0
#define XHELP_s_dot 1
#define XHELP_s_ddot 2
#define XHELP_phi 3
#define XHELP_phi_dot 4
#define XHELP_phi_ddot 5
#define XHELP_theta_ll 6
#define XHELP_theta_ll_dot 7
#define XHELP_theta_lr 8
#define XHELP_theta_lr_dot 9
#define XHELP_theta_b 10
#define XHELP_theta_b_dot 11
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
#define P_LOCO



/** Constants for mechanical*/
#define NUM_MOTORS 6

/// @brief Manage all the balancing control 
class BalancingControl{
    private:
    /** Variables - Ref */
        float _x_d[12]; 
        float _psi_d; 
        float _l_d; 
    /** Variables - fdb */
        float _x[12]; 
        float _psi; 
        float _ll; // Left Leg length 
        float _lr; // Right Leg length 
        float _jl[4]; 
        float _jr[4];
        float _a_z; 
    /** Helping Classes */
        Timer timer;
        /// @brief The PID for psi
        PIDFilter pid1; 
        /// @brief The PID for l
        PIDFilter pid2; 

    public:
        float output[NUM_MOTORS];
        /// @brief defalt constrctor 
        BalancingControl();


        void init();
        /// @brief ref input(Not sure if those are constant)
        /// @param x_d = [s, s_dot, s_ddot, phi, phi_dot, phi_ddot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot]
        /// @param psi_d 
        /// @param l_d leg length
        void set_refinput(float x_d[12], float psi_d, float l_d);

        /// @brief fdb input from the observer
        /// @param x = [s, s_dot, s_ddot, phi, phi_dot, phi_ddot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot]
        /// @param psi 
        /// @param ll left leg length
        /// @param lr right leg length
        /// @param jl jacobian matrix for left
        /// @param jr Jacobian matrix for right
        /// @param a_z 
        void set_fdbinput(float x[12], float psi, float ll, float lr, float jl[4], float jr[4], float a_z);
        /// @brief calculate the output and send them to the Can_bus
        /// @param output for balancing contorl the data form is [T_lwl, T_jlf, T_jlb, T_lwr, T_jrf, T_jrb] 
        void step(float output[NUM_MOTORS]);
};

#endif