#include "test_balancing.hpp"

void balancing_test::init(){
    slowdalay_help = micros();

    saftymode = 0;
    o_data.b_speed_old = 0;


    //PID1:roll, PID2:Leg length
    float gain1[4] = {K1_P, K1_I, K1_D, K1_F};
    float gain2[4] = {K2_P, K2_I, K2_D, K2_F};
    pid1.set_K(gain1);
    pid2.set_K(gain2);

    // p matrix for average leg length
    float tempp[P_LOCO_ROW][4][10] = {
        {
            {-0.079813, -79.825508, -0.023347, -1.172826, -40.369902, -2.339875, -85.840359, 1.076255, -10.232493, -1.059097, },
            {-0.079813, -79.825508, 0.023347, 1.172826, -85.840359, 1.076255, -40.369902, -2.339875, -10.232493, -1.059097, },
            {0.014226, 14.217859, -0.113545, -5.721099, 179.967820, -1.404619, 24.015566, 2.090618, 41.450970, -50.445075, },
            {0.014226, 14.217859, 0.113545, 5.721099, 24.015566, 2.090618, 179.967820, -1.404619, 41.450970, -50.445075, },
        },
        {
            {0.077829, 77.841262, 0.009984, 0.500776, 52.378022, 2.246133, 79.367622, -1.450668, 6.580985, -1.241988, },
            {0.077829, 77.841262, -0.009984, -0.500776, 79.367622, -1.450668, 52.378022, 2.246133, 6.580985, -1.241988, },
            {0.019619, 19.631535, 0.141142, 7.107154, -175.158910, 3.195682, 35.615948, -2.701015, -32.831216, 49.770164, },
            {0.019619, 19.631535, -0.141142, -7.107154, 35.615948, -2.701015, -175.158910, 3.195682, -32.831216, 49.770164, },
        },
        {
            {-0.027944, -27.950219, 0.007851, 0.395589, -36.566262, -3.908577, -27.354816, -1.520422, -0.255718, 1.992554, },
            {-0.027944, -27.950219, -0.007851, -0.395589, -27.354816, -1.520422, -36.566262, -3.908577, -0.255718, 1.992554, },
            {-0.029486, -29.495190, -0.056879, -2.862821, 51.569280, -2.516837, -44.853838, 1.389587, 8.791713, -17.980657, },
            {-0.029486, -29.495190, 0.056879, 2.862821, -44.853838, 1.389587, 51.569280, -2.516837, 8.791713, -17.980657, },
        },
        {
            {0.001207, 1.207155, -0.002927, -0.146968, -0.457798, -0.343497, -0.654306, -0.084102, -0.280002, -0.723001, },
            {0.001207, 1.207155, 0.002927, 0.146968, -0.654306, -0.084102, -0.457798, -0.343497, -0.280002, -0.723001, },
            {0.012313, 12.316690, -0.002415, -0.121774, 10.937257, 3.163992, 10.570094, 0.403461, -6.619837, 0.356433, },
            {0.012313, 12.316690, 0.002415, 0.121774, 10.570094, 0.403461, 10.937257, 3.163992, -6.619837, 0.356433, },
        },
    };    
    
    
    
    
    memcpy(p,tempp,sizeof(tempp));
    
    // For test purpose
    // float tempK[4][10] = { //18cm
    //     {-0.3157,-0.6659,-0.2234,-0.1460,-4.6021,-0.5745,-1.7850,-0.1971,-3.0104,-0.4326}, 
    //     {-0.3157,-0.6659,0.2234,0.1460,-1.7850,-0.1971,-4.6021,-0.5745,-3.0104,-0.4326}, 
    //     {0.0557,0.1116,-1.0806,-0.8358,6.7055,0.7714,-7.0740,-0.7067,-222.0533,-7.2328}, 
    //     {0.0557,0.1116,1.0806,0.8358,-7.0740,-0.7067,6.7055,0.7714,-222.0533,-7.2328}
    //     };
    // float tempK[4][10] = { //14cm
    //     {-0.3156, -0.6698, -0.2668, -0.1803, -4.2308, -0.5405, -1.9738, -0.2102, -3.4205, -0.5236}, 
    //     { -0.3156, -0.6698, 0.2668, 0.1803, -1.9738, -0.2102, -4.2308, -0.5405, -3.4205, -0.5236}, 
    //     {0.0647, 0.1300, -0.9773, -0.7482, 6.0704, 0.7614, -6.3280, -0.6935, -221.9400, -7.2531}, 
    //     {0.0647, 0.1300, 0.9773, 0.7482, -6.3280, -0.6935, 6.0704, 0.7614, -221.9400, -7.2531}
    //     };  

    // float tempK[4][10] = { //18cm
    //     {-0.4067, -1.0474,  0.2169,  0.2144, -6.9541, -0.7964,  0.3102, -0.0582,   7.6073, -0.1897}, 
    //     {-0.4067, -1.0474, -0.2169, -0.2144,  0.3102, -0.0582, -6.9541, -0.7964,   7.6073, -0.1897}, 
    //     {-0.0356, -0.0950, -0.4804, -0.3670,  1.6239,  0.1489, -3.0520, -0.2545, -74.4254, -3.1695}, 
    //     {-0.0356, -0.0950,  0.4804,  0.3670, -3.0520, -0.2545,  1.6239,  0.1489, -74.4254, -3.1695}
    //     };  

    // memcpy(K,tempK,sizeof(tempK));

    //ref for test
    _ref_data.goal_l = 0.18;
    _ref_data.goal_roll = 0;
    _ref_data.s = 0; 
    _ref_data.b_speed = 0;
    _ref_data.pitch = 0;
    _ref_data.pitch_dot = 0;
    _ref_data.theta_ll = 0;
    _ref_data.theta_ll_dot = 0;
    _ref_data.theta_lr = 0;
    _ref_data.theta_lr_dot = 0;
    _ref_data.yaw_dot = 0;
}

void balancing_test::set_data(balancing_sensor_data data){ // Convert 65535 to randiance
    _data = data;
    _data.angle_fr = _data.angle_fr / 182.04166666666666666666666666667;
    _data.angle_fr -= 9.5431;
    _data.angle_fl = _data.angle_fl / 182.04166666666666666666666666667;
    _data.angle_fl += 9.5431;
    _data.angle_bl = _data.angle_bl / 182.04166666666666666666666666667;
    _data.angle_bl -= 9.5431;
    _data.angle_br = _data.angle_br / 182.04166666666666666666666666667;
    _data.angle_br += 9.5431;

    _data.angle_fr *= DEG_TO_RAD;
    _data.angle_fl *= DEG_TO_RAD;
    _data.angle_bl *= DEG_TO_RAD;
    _data.angle_br *= DEG_TO_RAD;
}

void balancing_test::limit_write(){

}

void balancing_test::test_write(){
}


void balancing_test::observer(){
    _dt = timer.delta();
//---------------------------------------------------------Left Leg Forward Kinematics & Jacobian--------------------------------------------------------------
    float phi4_l =   (2 * M_PI - _data.angle_fl); 
    float phi1_l = (M_PI - _data.angle_bl); 
    float cphi1_l = cos(phi1_l ); //pi/2 - M3
    float sphi1_l = sin(phi1_l);
    float cphi4_l = cos(phi4_l); //2*pi - M2
    float sphi4_l = sin(phi4_l);

    float xBl = l_u * cphi1_l; 
    float yBl = l_u * sphi1_l;
    float xDl = 2 * l_a + l_u * cphi4_l;
    float yDl = l_u * sphi4_l;
    float lBDl = sqrt((xBl - xDl) * (xBl - xDl) + (yBl - yDl) * (yBl - yDl));
    float A0l = 2 * (xBl - xDl) * l_l;
    float b0l = 2 * (yBl - yDl) * l_l;
    float C0l = -(lBDl * lBDl);

    float phihelpl = sqrt(b0l * b0l + A0l * A0l - C0l * C0l);
    float phi2l= 2 * atan2(b0l - phihelpl, C0l + A0l);
    float phi3l= 2 * atan2(phihelpl - b0l, C0l - A0l);
    float xCl = l_u * cphi1_l + l_l * cos(phi2l);
    float yCl = l_u * sphi1_l+ l_l * sin(phi2l);

    float helpingl = (xCl - l_a);
    o_data.ll = sqrt(yCl * yCl + helpingl * helpingl); //ll
    float phi0l = atan2(yCl, helpingl); //phi 0

    o_data.jl[0][0] = (l_u * sin(phi0l - phi3l) * sin(phi1_l - phi2l)) / sin(phi2l - phi3l); 
    o_data.jl[0][1]= -(l_u * cos(phi0l - phi3l) * sin(phi1_l - phi2l)) / (o_data.ll * sin(phi2l - phi3l));
    o_data.jl[1][0] = (l_u * sin(phi0l - phi2l) * sin(phi3l - phi4_l)) / sin(phi2l - phi3l);
    o_data.jl[1][1] = -(l_u * cos(phi0l - phi2l) * sin(phi3l - phi4_l)) / (o_data.ll * sin(phi2l - phi3l));

    o_data.theta_ll = (-fmod((M_PI_2 - phi0l - _data.imu_angle_pitch), 2 * M_PI)); // This is the correct one 
 //----------------------------------------------------Right Leg Forward Kinematics & Jacobian--------------------------------------------------
    float phi4_r = _data.angle_fr;
    float phi1_r = (_data.angle_br - M_PI); 
    float cphi1r = cos(phi1_r); 
    float sphi1r = sin(phi1_r);
    float cphi4r = cos(phi4_r);
    float sphi4r = sin(phi4_r);

    float xBr = l_u * cphi1r;
    float yBr = l_u * sphi1r;
    float xDr = 2 * l_a + l_u * cphi4r;
    float yDr = l_u * sphi4r;
    float lBDr = sqrt((xBr - xDr) * (xBr - xDr) + (yBr - yDr) * (yBr - yDr));

    float a0r = 2 * (xBr - xDr) * l_l;
    float b0r = 2 * (yBr - yDr) * l_l;
    float c0r = -(lBDr * lBDr);    

    float phihelpr = sqrt(b0r * b0r + a0r * a0r - c0r * c0r);
    float phi2r= 2 * atan2(b0r - phihelpr, c0r + a0r);
    float phi3r= 2 * atan2(phihelpr - b0r, c0r - a0r);

    float xCr = l_u * cphi1r + l_l * cos(phi2r);
    float yCr = l_u * sphi1r + l_l * sin(phi2r);

    float helpingr = xCr - l_a;
    o_data.lr = sqrt(yCr * yCr + helpingr * helpingr); 
    float phi0r = atan2(yCr, helpingr);

    o_data.jr[0][0]  = (l_u * sin(phi0r - phi3r) * sin(phi1_r - phi2r)) / sin(phi2r - phi3r); 
    o_data.jr[0][1]  = -(l_u * cos(phi0r - phi3r) * sin(phi1_r - phi2r)) / (o_data.lr * sin(phi2r - phi3r));
    o_data.jr[1][0]  = (l_u * sin(phi0r - phi2r) * sin(phi3r - phi4_r)) / sin(phi2r - phi3r);
    o_data.jr[1][1]  = -(l_u * cos(phi0r - phi2r) * sin(phi3r - phi4_r)) / (o_data.lr * sin(phi2r - phi3r));

    o_data.theta_lr = (-fmod((M_PI_2 - phi0r - _data.imu_angle_pitch), (2 * M_PI)));
    

//----------------------------------------------------Calculate Theta_of_leg_dot for both leg--------------------------------------
    float phi2_dot_l = (l_u *((_data.speed_bl * sphi1_l - _data.speed_fl * sphi4_l) * cos(phi3l) + ((-_data.speed_bl) * cphi1_l + _data.speed_fl * cphi4_l) * sin(phi3l))) / (l_l * sin(phi2l - phi3l));
    float xC_dot_l = -(l_u * (-_data.speed_bl) * sphi1_l + l_l * phi2_dot_l * sin(phi2l));
    float yC_dot_l = (l_u * (-_data.speed_bl) * cphi1_l + l_l * phi2_dot_l * cos(phi2l));
    o_data.theta_ll_dot = (yC_dot_l*helpingl - xC_dot_l*yCl) < 0 ? (sqrt(xC_dot_l * xC_dot_l + yC_dot_l * yC_dot_l)/sqrt(helpingl*helpingl + yCl*yCl)) - _data.gyro_pitch : -(sqrt(xC_dot_l * xC_dot_l + yC_dot_l * yC_dot_l)/sqrt(helpingl*helpingl + yCl*yCl)) - _data.gyro_pitch;
    o_data.theta_ll_dot *= -1;
    o_data.ll_dot = sqrt(xC_dot_l * xC_dot_l + yC_dot_l * yC_dot_l);
    // o_data.theta_ll_dot = -(((helpingl*yC_dot_l) + (yCl * xC_dot_l)) / (helpingl * helpingl + yCl * yCl)) - _data.gyro_pitch;

    float phi2_dot_r = (l_u * ((-_data.speed_br * sphi1r + _data.speed_fr * sphi4r) * cos(phi3r) + (_data.speed_br * cphi1r - _data.speed_fr * cphi4r) * sin(phi3r))) / (l_l * sin(phi2r - phi3r));
    float xC_dot_r = -(l_u * _data.speed_br * sphi1r + l_l * phi2_dot_r * sin(phi2r));
    float yC_dot_r = (l_u * _data.speed_br * cphi1r + l_l * phi2_dot_r * cos(phi2r));
    o_data.theta_lr_dot = (yC_dot_r*helpingr - xC_dot_r*yCr) < 0 ? (sqrt(xC_dot_r * xC_dot_r + yC_dot_r * yC_dot_r)/sqrt(helpingr*helpingr + yCr*yCr)) - _data.gyro_pitch : -(sqrt(xC_dot_r * xC_dot_r + yC_dot_r * yC_dot_r)/sqrt(helpingr*helpingr + yCr*yCr)) - _data.gyro_pitch;
    o_data.theta_lr_dot *= -1;
    o_data.lr_dot = sqrt(xC_dot_r * xC_dot_r + yC_dot_r * yC_dot_r);
    // o_data.theta_lr_dot = -(((helpingr*yC_dot_r) + (yCr * xC_dot_r)) / (helpingr * helpingr + yCr * yCr)) - _data.gyro_pitch;
//--------------------------------------------------------------b_s and filter for it--------------------------------------------------------
    o_data.b_speed =  (1/2) * (R_w) * (_data.speed_wr - _data.speed_wl) ; // s_dot //speed 
//- (1/2) * (o_data.ll*(o_data.theta_ll_dot + _data.imu_angle_pitch)*cos(phi0l) + o_data.lr*(o_data.theta_lr_dot + _data.imu_angle_pitch)*cos(phi0r)) - (1/2)* (o_data.ll_dot * sin(phi0l) + o_data.lr_dot * sin(phi0r))
//-------------------------------------------filter by a falman filter (I will update this soon) ---------------------------------------------------------------------
    // For the x we have [v,a]^T 
    // predict


    return;
}


void balancing_test::control(){
        /** Initialize all output variables */
    _write.torque_fr = 0;
    _write.torque_fl = 0;
    _write.torque_bl = 0;
    _write.torque_br = 0;
    _write.torque_wl = 0;
    _write.torque_wr = 0;

//----------------------------------------------------------------leg_controller---------------------------------------------------------------------
    float l = (o_data.ll + o_data.lr) / 2; // Get the average leg length 
    float F_psi = pid1.filter(_dt, NOBOUND, WARP, _ref_data.goal_roll, _data.gyro_roll); //Set the PID for roll angle
    float F_l = pid2.filter(_dt, NOBOUND, NOWARP, _ref_data.goal_l, l); //Set the PID for leg length 

    float iF_r = ((((m_b / 2) + (eta_l * m_l)) * l * _data.gyro_pitch * o_data.b_speed) / 2) / R_l;
    float iF_l = -iF_r; 
//-----------------------------------------------------------------gravity feed forware-------------------------------------------------------------------
    float costheta_l = cos(o_data.theta_ll);
    float costheta_r = cos(o_data.theta_lr);


    float GF_help = (m_b / 2 + m_l * eta_l) * G_CONSTANT;
    float gF_l = GF_help * costheta_l;
    float gF_r = GF_help * costheta_r; 

    float F_bll = F_psi + F_l + iF_l + gF_l;
    float F_blr = -F_psi + F_l + iF_r + gF_r; 


    
//---------------------------------------------------------------The NormalF Left------------------------------------------------------------------------
    float F_whl = F_bll * costheta_l + m_l * ( G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.ll_ddot * costheta_l);

//----------------------------------------------------------------The NormalF Right----------------------------------------------------------------------- 
    float F_whr = F_blr * costheta_r + m_l * ( G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.lr_ddot * costheta_r);

    // do nothing check
    if(F_whl > F_WH_OUTPUT_LIMIT_NUM && F_whr > F_WH_OUTPUT_LIMIT_NUM)
        return;

//---------------------------------------------------------------locomotion_controller--------------------------------------------------------------------------
//--------------------------------------------------------------Acceleration Saturation---------------------------------------------------------------------------
    float dx[10];
    // dx[0] = 0; //Ignore // s
    dx[0] = _ref_data.s- o_data.s; // s
    dx[1] = _ref_data.b_speed - o_data.b_speed; // speed
    dx[2] = 0; //Ignore // yaw
    //dx[2] = ref[0][2] - obs[0][2]; // yaw angle //We don't have this data and don't need it
    dx[3] = _ref_data.yaw_dot - _data.gyro_yew; // yaw rotational speed in deg
    dx[4] = _ref_data.theta_ll - o_data.theta_ll; // theta_ll
    dx[5] = _ref_data.theta_ll_dot - o_data.theta_ll_dot; // theta_ll_dot
    dx[6] = _ref_data.theta_lr - o_data.theta_lr; // theta_lr
    dx[7] = _ref_data.theta_lr_dot - o_data.theta_lr_dot; // theta_lr_dot
    dx[8] = _ref_data.pitch - _data.imu_angle_pitch; // pitch
    dx[9] = _ref_data.pitch_dot - _data.gyro_pitch; // pitch_dot

//----------------------------------------------------------------Leg Length to K------------------------------------------------------------------------------
    // K from Full scale LQR
    // for(int i = 0; i < 4; i++) 
    //     for(int j = 0; j < 10; j++) 
    //         K[i][j] = p[0][i][j] * o_data.ll * o_data.ll + p[1][i][j] * o_data.ll * o_data.lr + p[2][i][j] * o_data.ll + p[3][i][j] * o_data.lr * o_data.lr + p[4][i][j] * o_data.lr + p[5][i][j];
    // K is 4x10 matrix 
    // 4x10 x 10x1 matrix multi (K * dx) 

    //K from single leg Length LQR
    for(int i = 0; i < 4; i++) 
        for(int j = 0; j < 10; j++) 
            K[i][j] = p[0][i][j] * l * l * l + p[1][i][j] * l * l + p[2][i][j] * l + p[3][i][j];

    
    float T_bll = 0;  
    float T_blr = 0;

    for(int i = 0; i < 10; i++){
        _write.torque_wl += K[0][i] * dx[i];
        _write.torque_wr -= K[1][i] * dx[i];
        T_bll += K[2][i] * dx[i];
        T_blr += K[3][i] * dx[i];
    } 


    // // wheels limit //May not be used
    // if(output[T_LWL_OUTPUT_NUM] > WHEEL_UPPER_LIMIT) output[T_LWL_OUTPUT_NUM] = WHEEL_UPPER_LIMIT;
    //     else if(output[T_LWL_OUTPUT_NUM] < WHEEL_LOWER_LIMIT) output[T_LWL_OUTPUT_NUM] = WHEEL_LOWER_LIMIT;
    // if(output[T_LWR_OUTPUT_NUM] > WHEEL_UPPER_LIMIT) output[T_LWR_OUTPUT_NUM] = WHEEL_UPPER_LIMIT;
    //     else if(output[T_LWR_OUTPUT_NUM] < WHEEL_LOWER_LIMIT) output[T_LWR_OUTPUT_NUM] = WHEEL_LOWER_LIMIT;

    //2x2 * 2x1 
    // jl[a][b]   *   F_bll
    //   [c][d]       T_bll
    // = [af + bt]
    //   [cf + dt]
//--------------------------------------------------------------Left side force---------------------------------------------------------------------------------
    if(F_whl <= F_WH_OUTPUT_LIMIT_NUM){
        _write.torque_fl = o_data.jl[0][0] * F_bll + o_data.jl[0][1] * T_bll;
        _write.torque_bl = o_data.jl[1][0] * F_bll + o_data.jl[1][1] * T_bll;
    }else{
        _write.torque_wl = 0;
    }
//-----------------------------------------------------------Right side check force------------------------------------------------------------------------------
    if(F_whr <= F_WH_OUTPUT_LIMIT_NUM){
        _write.torque_fr = o_data.jr[0][0] * F_blr + o_data.jr[0][1] * T_blr;
        _write.torque_br = o_data.jr[1][0] * F_blr + o_data.jr[1][1] * T_blr;
    }else{
        _write.torque_wr = 0;
    }
//-------------------------------------------------------------Calculate CAN value-----------------------------------------------
    
    
    if(_write.torque_wl > WHEEL_MOTOR_limit)
    _write.torque_wl = WHEEL_MOTOR_limit;
    if(_write.torque_wl < -WHEEL_MOTOR_limit)
    _write.torque_wl = -WHEEL_MOTOR_limit;
    _write.torque_wl /= 5.0;
    
    if(_write.torque_wr > WHEEL_MOTOR_limit)
    _write.torque_wr = WHEEL_MOTOR_limit;
    if(_write.torque_wr < -WHEEL_MOTOR_limit)
    _write.torque_wr = -WHEEL_MOTOR_limit;
    _write.torque_wr /= 5.0;
    
    if(_write.torque_fl > MGlimit)
    _write.torque_fl = MGlimit;
    if(_write.torque_fl < -MGlimit)
    _write.torque_fl = -MGlimit;
    _write.torque_fl /= -37.0;

    if(_write.torque_bl > MGlimit)
    _write.torque_bl = MGlimit;
    if(_write.torque_bl < -MGlimit)
    _write.torque_bl = -MGlimit;
    _write.torque_bl /= -37.0;

    if(_write.torque_fr > MGlimit)
    _write.torque_fr = MGlimit;
    if(_write.torque_fr < -MGlimit)
    _write.torque_fr = -MGlimit;
    _write.torque_fr /= 37.0;
    
    if(_write.torque_br > MGlimit)
    _write.torque_br = MGlimit;
    if(_write.torque_br < -MGlimit)
    _write.torque_br = -MGlimit;
    _write.torque_br /= 37.0;
return;
}

write_data balancing_test::getwrite(){

//-----------------------------------------------------------Safety limit--------------------------------------------------------------------
    // if(o_data.theta_ll > 0.8 || o_data.theta_ll < -0.8 || o_data.theta_lr > 0.8 || o_data.theta_ll < -0.8){
    //     _write.torque_fl = 0;
    //     _write.torque_bl = 0;
    //     _write.torque_wl = 0;
    //     _write.torque_fl = 0;
    //     _write.torque_bl = 0;
    //     _write.torque_wl = 0;
    //     saftymode = true;
    // }
    return _write;
}

void balancing_test::printdata(){
    getwrite();
    Serial.print("torque_fr: ");
    Serial.println(_write.torque_fr * 37);
    Serial.print("torque_fl: ");
    Serial.println(_write.torque_fl * 37);
    Serial.print("torque_bl: ");
    Serial.println(_write.torque_bl * 37);
    Serial.print("torque_br: ");
    Serial.println(_write.torque_br * 37);
    Serial.print("torque_wl: ");
    Serial.println(_write.torque_wl * 5);
    Serial.print("torque_wr: ");
    Serial.println(_write.torque_wr * 5);
}
void balancing_test::print_observer(){
    Serial.print("speed_wl ");
    Serial.println(_data.speed_wl);
    Serial.print("speed_wr ");
    Serial.println(_data.speed_wr);
    Serial.print("s: ");
    Serial.println(o_data.s);
    Serial.print("b_speed: ");
    Serial.println(o_data.b_speed);
    Serial.print("imu_s: ");
    Serial.println(o_data.imu_s);
    Serial.print("imu_speed: ");
    Serial.println(o_data.imu_speed_x);
    Serial.print("b_accel: ");
    Serial.println(o_data.b_accel);
    Serial.print("leglength ll: ");
    Serial.printf("%f", o_data.ll);
    Serial.println();
    Serial.print("leglength lr: ");
    Serial.printf("%f", o_data.lr);
    Serial.println();
    Serial.print("ll_ddot: ");
    Serial.println(o_data.ll_ddot);
    Serial.print("lr_ddot: ");
    Serial.println(o_data.lr_ddot);
    Serial.print("theta_ll: ");
    Serial.printf("%f", o_data.theta_ll*RAD_TO_DEG);
    Serial.println();
    Serial.print("theta_lr: ");
    Serial.printf("%f", o_data.theta_lr*RAD_TO_DEG);
    Serial.println();
    Serial.print("theta_ll_dot: ");
    Serial.printf("%f", o_data.theta_ll_dot*RAD_TO_DEG);
    Serial.println();
    Serial.print("theta_lr_dot: ");
    Serial.printf("%f", o_data.theta_lr_dot*RAD_TO_DEG);
    Serial.println();
    Serial.print("jl: ");
    Serial.print(o_data.jl[0][0]);
    Serial.print(" ");
    Serial.println(o_data.jl[0][1]);
    Serial.print(o_data.jl[1][0]);
    Serial.print(" ");
    Serial.println(o_data.jl[1][1]);
    Serial.print("jr: ");
    Serial.print(o_data.jr[0][0]);
    Serial.print(" ");
    Serial.println(o_data.jr[0][1]);
    Serial.print(o_data.jr[1][0]);
    Serial.print(" ");
    Serial.println(o_data.jr[1][1]);
    Serial.printf("pitch: %f, roll: %f, yaw: %f", _data.imu_angle_pitch, _data.imu_angle_roll, _data.imu_angle_yaw);

}

void balancing_test::print_visual(){
    return ;
}