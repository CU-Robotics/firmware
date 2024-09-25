#include "Balancing_Observer.hpp"

void BalancingObserver::observer_init(){

}
//for x[12] = [s 0, s_dot 1, phi 2, phi_dot 3, theta_ll 4, theta_ll_dot 5, theta_lr 6, theta_lr_dot 7, theta_b 8, theta_b_dot 9, s_ddot 10, phi_ddot 11] 
//{x[12] = obs[0-11], psi = obs[12], ll = obs[13], lr = obs[14], jl[0][0] = obs[15], jl[0][1] = obs[16], jl[1][1] = obs[17], jl[1][1] = obs[18], jr[0][0] = obs[19], jr[0][1] = obs[20], jr[1][0] = obs[21], jr[1][1] = obs[22], a_z = obs[23], ll_ddot = obs[24], lr_ddot = obs[25]}
void BalancingObserver::step(CANData* can, IMUData* imu, float obs[26]){
    // not used but needed
    float a_z; //out[25]
    //a_x is not used in anywhere
    float dt = timer.delta();
    // From sensors
    obs[8] = imu->gyro_X;// theta_b                 // need check 
    obs[9] = (obs[8] - _theta_b_old)/ dt;// theta_b_dot
    _theta_b_old = obs[8];

    obs[2] = imu->gyro_Z; // phi                    // need check
    obs[3] = (obs[2] - _phi_old)/ dt; // phi_dot    // need check
    _phi_old = obs[2];
    obs[12] = imu->gyro_Y;// psi                    // need check 

    obs[25] = imu->accel_Z;// a_z                    // need check

    
    float theta_wl; // need check
    float theta_wl_dot = can->get_motor_attribute(L_CAN, L_W_MOTORID, SPEED);
    float theta_wr; // need check
    float theta_wr_dot = can->get_motor_attribute(R_CAN, R_W_MOTORID, SPEED);


    float phi4_l = can->get_motor_attribute(L_CAN, L_FJ_MOTORID, ANGLE);// need check
    float phi1_l = can->get_motor_attribute(L_CAN, L_BJ_MOTORID, ANGLE);// need check
    float phi4_r = can->get_motor_attribute(R_CAN, R_FJ_MOTORID, ANGLE);// need check
    float phi1_r = can->get_motor_attribute(R_CAN, R_BJ_MOTORID, ANGLE);// need check


    obs[0] =  (1/2) * (R_w) * (theta_wl + theta_wr); // s // need check
    obs[1] =  (1/2) * (R_w) * (theta_wl_dot + theta_wr_dot); // s_dot

    





    //to get
    float theta_ll;
    float theta_ll_dot;
    float theta_lr;
    float theta_lr_dot;
}