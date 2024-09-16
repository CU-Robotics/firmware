#include "Balancing_Control.hpp"

BalancingControl::BalancingControl(){}

void BalancingControl::init(){
    pid1.set_K({K1_P, K1_I, K1_D, K1_F});
    pid2.set_K({K2_P, K2_I, K2_D, K2_F});

}

void BalancingControl::set_refinput(float x_d[12], float psi_d, float l_d){

}

void BalancingControl::set_fdbinput(float x[12], float psi, float ll, float lr, float jl[4], float jr[4], float a_z){

}

void BalancingControl::step(float output[NUM_MOTORS]){
    float dt = timer.delta(); 

    float l = (_ll + _lr) / 2; 
    /** This is the part for leg_controller */
        float F_psi = pid1.filter(dt, BOUND, WARP) * (_psi_d - _psi); //Will comment this 
        float F_l = pid2.filter(dt, BOUND, WARP) * (_l_d - l); // Will comment this

        /** In inertia_ff */
            //s_dot = _x[XHELP_s_dot], s_ddot = _x[XHELP_s_ddot], phi_dot = _x[XHELP_phi_dot], phi_ddot = _x[XHELP_phi_ddot], theta_ll = _x[XHELP_theta_ll], theta_lr = _x[XHELP_theta_lr]
            float iffhelp = (m_b / 2 + m_l * eta_l) * (_x[XHELP_phi_ddot] * R_l + _x[XHELP_s_ddot]);
            float iF_l = -iffhelp * sin(_x[XHELP_theta_ll]);
            float iF_r = iffhelp * sin(_x[XHELP_theta_lr]); 
            /* F = (m_b / 2 + eta_l * m_l) * l * _x[XHELP_phi_dot]* _x[XHELP_s_dot] / 2 / R_l;
            float Fr = F;
            float Fl = -Fr; */

        /** In gravity_ff */
            float gffhelp = (m_b / 2 + m_l * THE_C_IDK) * G_CONSTANT;
            float gF_l = gffhelp * cos(_x[XHELP_theta_ll]);
            float gF_r = gffhelp * cos(_x[XHELP_theta_lr]);

        float F_bll = F_psi * MA0 + F_l * MA1 + iF_l * MA2 + iF_r * MA3 + gF_l * MA4 + gF_r * MA5; //ASK for if the matrix will change or not
        float F_blr = F_psi * MA0 + F_l * MA1 + iF_l * MB2 + iF_r * MB3 + gF_l * MB4 + gF_r * MB5;

    /** This is the part for locomotion_controller */
        /** In Acceleration Saturation */
            // dx is a array but only used the element 1 in matlab which is 0 here
            float dx[12];
            for(int i = 0; i < 12; i++){
                dx[i] = _x_d[i] - _x[i];
            }
            if(dx[XHELP_s] < 1 && dx[XHELP_s] > -1)
                dx[XHELP_s] = fabs((dx[XHELP_s]) / (dx[XHELP_s]));
        /** In Leg Length to K */
            // K = p(:,:,1)*l_l^2 + p(:,:,2)*l_l * l_r + p(:,:,3)*l_l + p(:,:,4)*l_r^2 + p(:,:,5)*l_r+ p(:,:,6);

            // K = p;

            // l = (l_l+l_r)/2;
            // K = p(:,:,1)*l^3 + p(:,:,2)*l^2 + p(:,:,3)*l + p(:,:,4);
            

            
            

}   