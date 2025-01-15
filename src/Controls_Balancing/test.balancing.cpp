#include "test_balancing.hpp"

void balancing_test::init(){
    o_data.pitch_dot  = 0;
    o_data.yaw_dot = 0;
    o_data.yaw_ddot = 0;


    o_data.pitch_old = 0;
    o_data.yaw_dot_old = 0;
    o_data.theta_ll_old = 0;
    o_data.theta_lr_old = 0;
    o_data.b_speed_old = 0;
    o_data.ll_old = 0;
    o_data.ll_dot_old = 0;
    o_data.lr_old = 0;
    o_data.lr_dot_old = 0;


    float gain1[4] = {K1_P, K1_I, K1_D, K1_F};
    float gain2[4] = {K2_P, K2_I, K2_D, K2_F};
    pid1.set_K(gain1);
    pid2.set_K(gain2);
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
    _data.angle_fr = _data.angle_fr / 182.04166666666666666666666666667;
    _data.angle_fl = _data.angle_fl / 182.04166666666666666666666666667;
    _data.angle_br = _data.angle_br / 182.04166666666666666666666666667;
    _data.angle_bl = _data.angle_bl / 182.04166666666666666666666666667;
}

void balancing_test::limit_write(){

}

void balancing_test::test_write(){
}


void balancing_test::observer(){
    float dt = timer.delta();
//---------------------------------------------------------------------- Data from sensors---------------------------------------------------------
    //_data.imu_angle_pitch                                     // pitch
    //_data.imu_angle_roll;                                     // roll   
    // _data.imu_accel_z;                                       // a_z
    //_data.speed_wl;                                           // wheel speed left
    //_data.speed_wr;                                           // wheel speed right

    //_data.angle_bl                                            //joint angle back left  id:3 
    //_data.angle_fl                                            //joint angle front left id:2
    //_data.angle_br                                            //joint angle back right id:4 
    //_data.angle_fr                                            //joint angle front right id:1
       
    o_data.pitch_dot = (_data.imu_angle_pitch  - o_data.pitch_old) / dt;        // pitch_dot
    o_data.yaw_dot = _data.gyro_yew;                                                // yaw_dot    
    o_data.yaw_ddot = (o_data.yaw_dot - o_data.yaw_dot_old) / dt;                   // yaw_ddot 

    o_data.pitch_old = _data.imu_angle_pitch; 
    o_data.yaw_dot_old = o_data.yaw_dot;

    o_data.b_speed =  (1.0f/2) * (R_w) * (_data.speed_wl + _data.speed_wr); // s_dot //speed
    o_data.b_accel = (o_data.b_speed - o_data.b_speed_old) / dt; //s_ddot //acceleration


//---------------------------------------------------------Left Leg Forward Kinematics & Jacobian--------------------------------------------------------------
    float phi4_l = _data.angle_bl * DEG_TO_RAD;
    float phi1_l = (_data.angle_fl - 180) * DEG_TO_RAD; 

    float cphi1_l = cos(phi1_l ); //M2 - 180
    float sphi1_l = sin(phi1_l);
    float cphi4_l = cos(phi4_l); //M3 angle
    float sphi4_l = sin(phi4_l);

    float xBl = l_u * cphi1_l; 
    float yBl = l_u * sphi1_l;
    float xDl = 2 * l_a + l_u * cphi4_l;
    float yDl = l_u * sphi4_l;
    float lBDl = sqrt((xBl - xDl) * (xBl - xDl) + (yBl - yDl) * (yBl - yDl));
 
    float A0l = 2 * (xBl - xDl) * l_l;
    float b0l = 2 * (yBl - yDl) * l_l;
    float C0l = -(lBDl * lBDl);
    float phihelpl = sqrt(b0l * b0l + A0l * A0l - C0l * C0l) - b0l;
    float phi2l= 2 * atan2(phihelpl, A0l + C0l);
    phi2l= fmod(phi2l, 2 * M_PI);
    float phi3l= 2 * atan2(phihelpl, C0l - A0l);

    float xCl = l_u * cphi1_l + l_l * cos(phi2l);
    float yCl = l_u * sphi1_l+ l_l * sin(phi2l);
    //float xC_l = (l_u * cphi4_l + l_l * cos(phi3l) + 2 * l_a);

    float helpingl = (xCl - l_a);
    o_data.ll = sqrt(yCl * yCl + helpingl * helpingl);//ll
    float phi0l = atan2(yCl, helpingl);

    o_data.jl[0][0] = (l_u * sin(phi0l - phi3l) * sin(phi1_l - phi2l)) / sin(phi2l - phi3l); 
    o_data.jl[0][1]= -(l_u * cos(phi0l - phi3l) * sin(phi1_l - phi2l)) / (o_data.ll * sin(phi2l - phi3l));
    o_data.jl[1][0] = (l_u * sin(phi0l - phi2l) * sin(phi3l - phi4_l)) / sin(phi2l - phi3l);
    o_data.jl[1][1] = -(l_u * cos(phi0l - phi2l) * sin(phi3l - phi4_l)) / (o_data.ll * sin(phi2l - phi3l));

    o_data.theta_ll = fmod((M_PI_2 + _data.imu_angle_pitch - phi0l + M_PI), 2 * M_PI) - M_PI;
    
 //----------------------------------------------------Right Leg Forward Kinematics & Jacobian--------------------------------------------------
    float phi4_r = _data.angle_fr * DEG_TO_RAD;
    float phi1_r = (_data.angle_br - 180) * DEG_TO_RAD; 

    float cphi1r = cos(phi1_r); //M4 - 180
    float sphi1r = sin(phi1_r);
    float cphi4r = cos(phi4_r); //M1 angle
    float sphi4r = sin(phi4_r);

    float xBr = l_u * cphi1r;
    float yBr = l_u * sphi1r;
    float xDr = 2 * l_a + l_u * cphi4r;
    float yDr = l_u * sphi4r;
    float lBDr = sqrt((xBr - xDr) * (xBr - xDr) + (yBr - yDr) * (yBr - yDr));
 
    float a0r = 2 * (xBr - xDr) * l_l;
    float b0r = 2 * (yBr - yDr) * l_l;
    float c0r = -(lBDr * lBDr);
    float phihelpr = sqrt(b0r * b0r + a0r * a0r - c0r * c0r) - b0r;
    float phi2r= 2 * atan2(phihelpr, a0r + c0r);
    phi2r= fmod(phi2r, 2 * M_PI);
    float phi3r= 2 * atan2(phihelpr, c0r - a0r);

    float xCr = l_u * cphi1r + l_l * cos(phi2r);
    float yCr = l_u * sphi1r + l_l * sin(phi2r);
    //float xC_r = (l_u * cphi4r + l_l * cos(phi3r) + 2 * l_a);

    float helpingr = xCr - l_a;
    o_data.lr = sqrt(yCr * yCr + helpingr * helpingr); // lr
    float phi0r = atan2(yCr, helpingr);

    o_data.jr[0][0]  = (l_u * sin(phi0r - phi3r) * sin(phi1_r - phi2r)) / sin(phi2r - phi3r); 
    o_data.jr[0][0]  = -(l_u * cos(phi0r - phi3r) * sin(phi1_r - phi2r)) / (o_data.lr * sin(phi2r - phi3r));
    o_data.jr[0][0]  = (l_u * sin(phi0r - phi2r) * sin(phi3r - phi4_r)) / sin(phi2r - phi3r);
    o_data.jr[0][0]  = -(l_u * cos(phi0r - phi2r) * sin(phi3r - phi4_r)) / (o_data.lr * sin(phi2r - phi3r));

    o_data.theta_lr = fmod((M_PI_2 + _data.imu_angle_pitch - phi0r + M_PI), (2 * M_PI)) - M_PI;
//-----------------------------------------------------------Get theta_ll_dot and theta_lr_dot-------------------------------------------------------
    o_data.theta_ll_dot = (o_data.theta_ll - o_data.theta_ll_old) / dt;
    o_data.theta_ll_old = o_data.theta_ll;
    o_data.theta_lr_dot = (o_data.theta_lr - o_data.theta_lr_old) / dt;
    o_data.theta_lr_old = o_data.theta_lr;

//-----------------------------------------------------------Get ll_ddot and lr_ddot-------------------------------------------------------------------
    float ll_dot = (o_data.ll - o_data.ll_old) / dt;
    o_data.ll_old = o_data.ll;
    o_data.ll_ddot = (ll_dot - o_data.ll_dot_old) / dt;
    o_data.ll_dot_old = ll_dot;

    float lr_dot = (o_data.lr - o_data.lr_old) / dt;
    o_data.lr_old = o_data.lr;
    o_data.lr_ddot = (lr_dot - o_data.lr_dot_old) / dt;
    o_data.lr_dot_old = lr_dot;
    return;
}

void balancing_test::control(){

}

write_data balancing_test::getwrite(){
    return _write;
}

void balancing_test::printdata(){
    Serial.println("Motor 1 angle");
    Serial.println(_data.angle_fr);
    Serial.println("Motor 2 angle");
    Serial.println(_data.angle_fl);
    Serial.println("Motor 3 angle");
    Serial.println(_data.angle_bl);
    Serial.println("Motor 4 angle");
    Serial.println(_data.angle_br);
}