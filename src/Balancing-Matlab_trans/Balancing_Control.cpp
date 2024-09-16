#include "Balancing_Control.hpp"

BalancingControl::BalancingControl(){}

void BalancingControl::init(){
    pid1.set_K({K1_P, K1_I, K1_D, K1_F});
    pid2.set_K({K2_P, K2_I, K2_D, K2_F});

}

void BalancingControl::set_refinput(float x_d[10], float psi_d, float l_d){

}

void BalancingControl::set_fdbinput(float x[10], float psi, float ll, float lr, float jl[4], float jr[4], float a_z){

}

void BalancingControl::step(float output[NUM_MOTORS]){
    float dt = timer.delta(); 

    // This is the part for leg_controller
    //l = (_ll + _lr)/2 
    //_psi_d - _psi need PID
    //_ld - l need PID
    //F_psi = pid1.filter(dt, BOUND, WARP);
    

}