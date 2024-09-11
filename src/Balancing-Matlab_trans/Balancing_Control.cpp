#include "Balancing_Control.hpp"

BalancingControl::BalancingControl(){}

void BalancingControl::set_fdbinput(float x_d[10], float psi_d, float l_d){

}

void BalancingControl::set_fdbinput(float x[10], float psi, float ll, float lr, float jl[4], float jr[4], float a_z){

}

void BalancingControl::step(){
    float dt = timer.delta();
}