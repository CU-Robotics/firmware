#ifndef BALANCING_CONTROL_H
#define BALANCING_CONTROL_H
#include "../comms/rm_can.hpp"
#include "../filters/pid_filter.hpp"
#include "../utils/timing.hpp"
/** Constants for math functions*/
#define param.m_b 
#define param.m_l 
#define param.R_l 
#define led_controller.eta_l
#define p 
/** Constants for mechanical*/
#define NUM_MOTORS 6

/// @brief Manage all the balancing control 
class BalancingControl{
    private:
    /** Variables - Ref */
        float _x_d[10]; //Checked 
        float _psi_d; // Checked 
        float _l_d; // Checked
    /** Variables - fdb */
        float _x[10]; // Checked
        float _psi; // Checked
        float _ll; // Left Leg length 
        float _lr; // Right Leg length 
        float _jl[4]; // Checked
        float _jr[4]; // Checked 
        float _a_z; //Checked
    /** Helping Classes */
        Timer timer;
        PIDFilter pid;

    public:
        float output[NUM_MOTORS];
        /// @brief defalt constrctor 
        BalancingControl();

        /// @brief ref input(Not sure if those are constant)
        /// @param x_d = [s, s_dot, phi, phi_dot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot]
        /// @param psi_d 
        /// @param l_d leg length
        void set_refinput(float x_d[10], float psi_d, float l_d);

        /// @brief fdb input from the observer
        /// @param x = [s, s_dot, phi, phi_dot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot]
        /// @param psi 
        /// @param ll left leg length
        /// @param lr right leg length
        /// @param jl jacobian matrix for left
        /// @param jr Jacobian matrix for right
        /// @param a_z 
        void set_fdbinput(float x[10], float psi, float ll, float lr, float jl[4], float jr[4], float a_z);
        /// @brief calculate the output and send them to the Can_bus
        /// @param output for balancing contorl the data form is [T_lwl, T_jlf, T_jlb, T_lwr, T_jrf, T_jrb] 
        void step(float output[NUM_MOTORS]);
};

#endif