#include "test_balancing.hpp"

void balancing_test::init(){
    float gain1[4] = {K1_P, K1_I, K1_D, K1_F};
    float gain2[4] = {K2_P, K2_I, K2_D, K2_F};
    pid1.set_K(gain1);
    pid2.set_K(gain2);
    // p[rows = P_LOCO_ROW] [cols = XHELP_LENGTH]
    float tempp[6][P_LOCO_ROW][10] = {{
        {0,0,0,0,0,0,0,0,0,0},  
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}},
        {
        {0,0,0,0,0,0,0,0,0,0},  
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}},
        {
        {0,0,0,0,0,0,0,0,0,0},  
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}},
        {
        {0,0,0,0,0,0,0,0,0,0},  
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}},
        {
        {0,0,0,0,0,0,0,0,0,0},  
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}},
        {
        {0,0,0,0,0,0,0,0,0,0},  
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}, 
        {0,0,0,0,0,0,0,0,0,0}
        }};
        
    memcpy(p,tempp,sizeof(tempp));
}

void balancing_test::set_data(balancing_sensor_data data){
    _data = data;
}

void balancing_test::limit_write(){
    // Check fixed limit
    if(_data.angle_fl > limit_fl && _torque_fl > 0){
        _torque_fl = 0;
    }
    if(_data.angle_fr > limit_fl && _torque_fr > 0){
        _torque_fr = 0;
    }
    if(_data.angle_bl > limit_fl && _torque_bl > 0){
        _torque_bl = 0;
    }
    if(_data.angle_br > limit_fl && _torque_br > 0){
        _torque_br = 0;
    }
    // Check relate limit
    
}

void balancing_test::observer(){

}

void balancing_test::control(){

}