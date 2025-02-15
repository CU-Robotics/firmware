#include "test_balancing.hpp"

void balancing_test::init(){
    slowdalay_help = micros();

    saftymode = 0;
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

    // p matrix for average leg length
    float tempp[P_LOCO_ROW][4][10] = {
        {
            {-73.848505, -151.089613, -6.068041, -1.952158, 10.911271, 13.838933, -18.107184, 13.119426, -1.021653, -16.582812, },
            {-73.848505, -151.089613, 6.068041, 1.952158, -18.107184, 13.119426, 10.911271, 13.838933, -1.021653, -16.582812, },
            {296.121464, 641.240903, -35.103478, -10.351650, 608.605484, 40.265488, 555.062183, 44.728448, -8.233883, -63.226927, },
            {296.121464, 641.240903, 35.103478, 10.351650, 555.062183, 44.728448, 608.605484, 40.265488, -8.233883, -63.226927, },
        },
        {
            {83.224686, 172.705821, 3.356445, 1.232907, 21.518723, -12.695385, 48.708160, -11.037238, -4.176761, 12.225869, },
            {83.224686, 172.705821, -3.356445, -1.232907, 48.708160, -11.037238, 21.518723, -12.695385, -4.176761, 12.225869, },
            {-233.399785, -510.677931, 47.922676, 14.199417, -597.683313, -48.768034, -498.077322, -53.404711, 5.193691, 70.379136, },
            {-233.399785, -510.677931, -47.922676, -14.199417, -498.077322, -53.404711, -597.683313, -48.768034, 5.193691, 70.379136, },
        },
        {
            {-35.649174, -76.448322, 2.091659, 0.662241, -48.608089, -3.491023, -44.048522, -3.085554, 4.224984, -1.593664, },
            {-35.649174, -76.448322, -2.091659, -0.662241, -44.048522, -3.085554, -48.608089, -3.491023, 4.224984, -1.593664, },
            {44.540363, 100.521896, -25.553965, -8.122012, 211.929224, 25.196332, 119.560228, 22.377394, 2.912889, -29.118372, },
            {44.540363, 100.521896, 25.553965, 8.122012, 119.560228, 22.377394, 211.929224, 25.196332, 2.912889, -29.118372, },
        },
        {
            {3.507067, 7.255959, -1.681419, -0.434956, 1.663422, 0.098835, 1.535239, 0.130738, -0.392370, -0.495047, },
            {3.507067, 7.255959, 1.681419, 0.434956, 1.535239, 0.130738, 1.663422, 0.098835, -0.392370, -0.495047, },
            {8.136661, 17.921959, -0.768818, -0.284471, 7.376691, 1.096476, 7.084931, 0.447353, -7.337519, 2.300633, },
            {8.136661, 17.921959, 0.768818, 0.284471, 7.084931, 0.447353, 7.376691, 1.096476, -7.337519, 2.300633, },
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

void balancing_test::set_data(balancing_sensor_data data){
    _data = data;
    _data.angle_fr = _data.angle_fr / 182.04166666666666666666666666667;
    _data.angle_fr -= 9.5431;
    _data.angle_fl = _data.angle_fl / 182.04166666666666666666666666667;
    _data.angle_fl += 9.5431;
    _data.angle_bl = _data.angle_bl / 182.04166666666666666666666666667;
    _data.angle_fl -= 9.5431;
    _data.angle_br = _data.angle_br / 182.04166666666666666666666666667;
    _data.angle_fr += 9.5431;
}

void balancing_test::limit_write(){

}

void balancing_test::test_write(){
}


void balancing_test::observer(){
    _dt = timer.delta();
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

    o_data.pitch_dot = _data.gyro_pitch > 0.02 || _data.gyro_pitch < -0.02 ? _data.gyro_pitch : 0; 
    // o_data.pitch_dot = (_data.imu_angle_pitch  - o_data.pitch_old) / _dt;        // pitch_dot
    o_data.yaw_dot = _data.gyro_yew > 0.02 || _data.gyro_yew < -0.02 ? _data.gyro_yew : 0;                                                // yaw_dot    
    o_data.yaw_ddot = (o_data.yaw_dot - o_data.yaw_dot_old) / _dt;                   // yaw_ddot 

    o_data.pitch_old = _data.imu_angle_pitch; 
    o_data.yaw_dot_old = o_data.yaw_dot; 

//---------------------------------------------------------Left Leg Forward Kinematics & Jacobian--------------------------------------------------------------
    float phi4_l =   (360 - _data.angle_fl)* DEG_TO_RAD;
    float phi1_l = (180 - _data.angle_bl) * DEG_TO_RAD; 
    float cphi1_l = cos(phi1_l ); //180 - M3
    float sphi1_l = sin(phi1_l);
    float cphi4_l = cos(phi4_l); //360 - M2
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


    //o_data.theta_ll = (fmod((M_PI_2 + _data.imu_angle_pitch - phi0l + M_PI), 2 * M_PI) - M_PI); // original
    // o_data.theta_ll = (fmod((M_PI_2 + _data.imu_angle_pitch - phi0l), 2 * M_PI)); // simplfy
    o_data.theta_ll = (fmod((M_PI_2 - _data.imu_angle_pitch - phi0l), 2 * M_PI)); // This is the correct one 



    
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

    float phihelpr = sqrt(b0r * b0r + a0r * a0r - c0r * c0r);
    float phi2r= 2 * atan2(b0r - phihelpr, c0r + a0r);
    float phi3r= 2 * atan2(phihelpr - b0r, c0r - a0r);

    float xCr = l_u * cphi1r + l_l * cos(phi2r);
    float yCr = l_u * sphi1r + l_l * sin(phi2r);

    float helpingr = xCr - l_a;
    o_data.lr = sqrt(yCr * yCr + helpingr * helpingr); // lr
    float phi0r = atan2(yCr, helpingr);

    o_data.jr[0][0]  = (l_u * sin(phi0r - phi3r) * sin(phi1_r - phi2r)) / sin(phi2r - phi3r); 
    o_data.jr[0][1]  = -(l_u * cos(phi0r - phi3r) * sin(phi1_r - phi2r)) / (o_data.lr * sin(phi2r - phi3r));
    o_data.jr[1][0]  = (l_u * sin(phi0r - phi2r) * sin(phi3r - phi4_r)) / sin(phi2r - phi3r);
    o_data.jr[1][1]  = -(l_u * cos(phi0r - phi2r) * sin(phi3r - phi4_r)) / (o_data.lr * sin(phi2r - phi3r));

    o_data.theta_lr = (fmod((M_PI_2 - _data.imu_angle_pitch - phi0r), (2 * M_PI)));
    

//----------------------------------------------------Calculate Theta_of_leg_dot for both leg--------------------------------------
    float phi2_dot_l = (l_u * ((-_data.speed_bl * sphi1_l + _data.speed_fl * sphi4_l) * cos(phi3l) + (-_data.speed_fl * cphi4_l + _data.speed_bl * cphi1_l) * sin(phi3l))) / (l_l * (phi3l - phi2l));
    float xC_dot_l = -(l_u * -_data.speed_bl * sphi1_l + l_l * phi2_dot_l * sin(phi2l));
    float yC_dot_l = (l_u * -_data.speed_bl * cphi1_l + l_l * phi2_dot_l * cos(phi2l));
    o_data.theta_ll_dot = -(((helpingl*yC_dot_l) + (yCl * xC_dot_l)) / (helpingl * helpingl + yCl * yCl)) - _data.gyro_pitch;

    float phi2_dot_r = (l_u * ((_data.speed_br * sphi1r - _data.speed_fr * sphi4r) * cos(phi3r) + (_data.speed_fr * cphi4r - _data.speed_br * cphi1r) * sin(phi3r))) / (l_l * (phi3r - phi2r));
    float xC_dot_r = -(l_u * _data.speed_br * sphi1r + l_l * phi2_dot_r * sin(phi2r));
    float yC_dot_r = (l_u * _data.speed_br * cphi1r + l_l * phi2_dot_r * cos(phi2r));
    o_data.theta_lr_dot = -(((helpingr*yC_dot_r) + (yCr * xC_dot_r)) / (helpingr * helpingr + yCr * yCr)) - _data.gyro_pitch;
//Bad but useable? No it's not useable


    // o_data.theta_ll_dot = (o_data.theta_ll - o_data.theta_ll_old)   / _dt;
    // o_data.theta_lr_dot = (o_data.theta_lr - o_data.theta_lr_old) / _dt;
    // o_data.theta_ll_old = o_data.theta_ll;
    // o_data.theta_lr_old = o_data.theta_lr;
// //This is sooooo bad
//     o_data.theta_ll_avg += o_data.theta_ll;
//     o_data.theta_lr_avg += o_data.theta_lr;
//     o_data.avg_count += 1;

//     uint32_t timenow = millis();
//     float slowdt = timenow - slowdalay_help;
//     if(slowdt > 4 || slowdt < -100){
//         slowdt /= 1000; // To second
//         slowdalay_help = timenow;
//         o_data.theta_ll_avg /=  o_data.avg_count;
//         o_data.theta_lr_avg /=  o_data.avg_count;

// //----------------------------------------------------------Get theta_ll_dot and theta_lr_dot-------------------------------------------------------
//         o_data.theta_ll_dot = abs(o_data.theta_ll_avg - o_data.theta_ll_old) > THETA_FILTER  ? (o_data.theta_ll_avg - o_data.theta_ll_old) / slowdt : 0;
//         o_data.theta_ll_old = o_data.theta_ll_avg;
//         o_data.theta_lr_dot = abs(o_data.theta_lr_avg - o_data.theta_lr_old) > THETA_FILTER  ? (o_data.theta_lr_avg - o_data.theta_lr_old) / slowdt : 0;
//         o_data.theta_lr_old = o_data.theta_lr_avg;
// //-----------------------------------------------------------Get ll_ddot and lr_ddot-------------------------------------------------------------------
//         // float ll_dot = abs(o_data.llaverage - o_data.ll_old) > LL_FILTER ? (o_data.llaverage - o_data.ll_old) / slowdt : 0;
//         // o_data.ll_old = o_data.llaverage;
//         // o_data.ll_ddot = abs(ll_dot - o_data.ll_dot_old) > LL_FILTER ? (ll_dot - o_data.ll_dot_old) / slowdt : 0;
//         // o_data.ll_dot_old = ll_dot;

//         // float lr_dot = abs(o_data.lraverage - o_data.lr_old) > LL_FILTER ? (o_data.lraverage - o_data.lr_old) / slowdt : 0;
//         // o_data.lr_old = o_data.lraverage;
//         // o_data.lr_ddot = abs(lr_dot - o_data.lr_dot_old) > LL_FILTER ? (lr_dot - o_data.lr_dot_old) / slowdt : 0;
//         // o_data.lr_dot_old = lr_dot;


//         // o_data.llaverage = 0;
//         // o_data.lraverage = 0;
//         o_data.theta_ll_avg = 0;
//         o_data.theta_lr_avg = 0;
//         o_data.avg_count = 0;
//     }
//--------------------------------------------------------------b_s and filter for it--------------------------------------------------------
    _data.speed_wl /= -M3508RATIO;
    _data.speed_wr /= M3508RATIO;

    o_data.b_speed =  (1.0f/2) * (R_w) * (_data.speed_wr + _data.speed_wl); // s_dot //speed

//-------------------------------------------motion estimate and filter (I will update this soon) ---------------------------------------------------------------------
    float alpha1 = min(abs(0.3/o_data.b_speed), 1.0f) * 0.5; // 1st filter for using wheel data more when low speed
    o_data.imu_speed_x =  (alpha1 * o_data.b_speed) + ((1 - alpha1) * o_data.imu_speed_x); 
    
    float alpha = min((0.1 / ((abs(o_data.imu_speed_x) - abs(o_data.b_speed)))), 1.0f) * 0.005; // 2nd filter for using more imu data when both data's difference huge
    o_data.imu_speed_x =  (alpha * o_data.b_speed) + ((1 - alpha) * o_data.imu_speed_x);

    o_data.imu_speed_x += abs(_data.imu_accel_x) > 0.08 && abs(_data.imu_accel_x) < 1 ? _data.imu_accel_x * _dt : 0; // Not sure why but imu sometimes give crazy data

    o_data.s += o_data.b_speed * _dt; 
    o_data.imu_s += o_data.imu_speed_x * _dt;
    
    o_data.b_accel = (o_data.b_speed - o_data.b_speed_old) / _dt; //s_ddot //acceleration
    o_data.b_speed_old = o_data.b_speed;

    return;
}


void balancing_test::control_position(){
        /** Initialize all output variables */
    _write.torque_fr = 0;
    _write.torque_fl = 0;
    _write.torque_bl = 0;
    _write.torque_br = 0;
    _write.torque_wl = 0;
    _write.torque_wr = 0;

//----------------------------------------------------------------leg_controller---------------------------------------------------------------------
    float l = (o_data.ll + o_data.lr) / 2; // Get the average leg length 
    float F_psi = pid1.filter(_dt, BOUND, WARP) * (_ref_data.goal_roll - _data.gyro_roll); //Set the PID for roll angle
    float F_l = pid2.filter(_dt, BOUND, WARP) * (_ref_data.goal_l - l); //Set the PID for leg length 

    float iF_r = ((((m_b / 2) + (eta_l * m_l)) * l * o_data.pitch_dot * o_data.b_speed) / 2) / R_l;
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


    Serial.print("F_whl: ");
    Serial.printf("%E", F_whl);
    Serial.println();
    Serial.print("F_whr: ");
    Serial.printf("%E", F_whr);
    Serial.println();
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
    dx[3] = _ref_data.yaw_dot - o_data.yaw_dot; // yaw rotational speed in deg
    dx[4] = _ref_data.theta_ll - o_data.theta_ll; // theta_ll
    dx[5] = _ref_data.theta_ll_dot - o_data.theta_ll_dot; // theta_ll_dot
    dx[6] = _ref_data.theta_lr - o_data.theta_lr; // theta_lr
    dx[7] = _ref_data.theta_lr_dot - o_data.theta_lr_dot; // theta_lr_dot
    dx[8] = _ref_data.pitch - _data.imu_angle_pitch; // pitch
    dx[9] = _ref_data.pitch_dot - o_data.pitch_dot; // pitch_dot

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

    Serial.print("T_bll: ");
    Serial.printf("%E", T_bll);
    Serial.println();
    Serial.print("T_blr: ");
    Serial.printf("%E", T_blr);
    Serial.println();

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
    // float temp = _write.torque_fr;
    // _write.torque_fr = _write.torque_br; 
    // _write.torque_br = temp;
    // float temp = _write.torque_fl;
    // _write.torque_fl = _write.torque_bl; 
    // _write.torque_bl = temp;
    
    
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




// /// @brief Control part
// void balancing_test::control(){
//     /** Initialize all output variables */
//     _write.torque_fr = 0;
//     _write.torque_fl = 0;
//     _write.torque_bl = 0;
//     _write.torque_br = 0;
//     _write.torque_wl = 0;
//     _write.torque_wr = 0;

// //----------------------------------------------------------------leg_controller---------------------------------------------------------------------
//     float l = (o_data.ll + o_data.lr) / 2; // Get the average leg length 
//     float F_psi = pid1.filter(_dt, BOUND, WARP) * (_ref_data.goal_roll - _data.gyro_roll); //Set the PID for roll angle
//     float F_l = pid2.filter(_dt, BOUND, WARP) * (_ref_data.goal_l - l); //Set the PID for leg length 

//     float iF_r = ((((m_b / 2) + (eta_l * m_l)) * l * o_data.pitch_dot * o_data.b_speed) / 2) / R_l;
//     float iF_l = -iF_r; 

// //-----------------------------------------------------------------gravity force forware-------------------------------------------------------------------
//     float costheta_l = cos(o_data.theta_ll);
//     float costheta_r = cos(o_data.theta_lr);
//     float GF_help = (m_b / 2 + m_l * eta_l) * G_CONSTANT;
//     float gF_l = GF_help * costheta_l;
//     float gF_r = GF_help * costheta_r; 

//     float F_bll = F_psi + F_l + iF_l + gF_l;
//     float F_blr = -F_psi + F_l + iF_r + gF_r; 
    
// //---------------------------------------------------------------The NormalF Left------------------------------------------------------------------------
//     float F_whl = F_bll * costheta_l + m_l * (G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.ll_ddot * costheta_l);
// //----------------------------------------------------------------The NormalF Right----------------------------------------------------------------------- 
//     float F_whr = F_blr * costheta_r + m_l * (G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.lr_ddot * costheta_r);

//     // do nothing check
//     if(F_whl < F_WH_OUTPUT_LIMIT_NUM && F_whr < F_WH_OUTPUT_LIMIT_NUM)
//         return;

// //---------------------------------------------------------------locomotion_controller--------------------------------------------------------------------------
// //--------------------------------------------------------------Acceleration Saturation---------------------------------------------------------------------------
//     float dx[10];
//     dx[0] = 0; //Ignore // s
//     //dx[0] = ref[0][0] - obs[0][0]; // s-33A~33A
//     dx[1] = _ref_data.b_speed - o_data.b_speed; // s_dot //speed
//     dx[2] = 0; //Ignore // yaw
//     //dx[2] = ref[0][2] - obs[0][2]; // yaw angle //We don't have this data and don't need it
//     dx[3] = _ref_data.yaw_dot - o_data.yaw_dot; // yaw rotational speed
//     dx[4] = _ref_data.theta_ll - o_data.theta_ll; // theta_ll
//     dx[5] = _ref_data.theta_ll_dot - o_data.theta_ll_dot; // theta_ll_dot
//     dx[6] = _ref_data.theta_lr - o_data.theta_lr; // theta_lr
//     dx[7] = _ref_data.theta_lr_dot - o_data.theta_lr_dot; // theta_lr_dot
//     dx[8] = _ref_data.pitch - _data.gyro_pitch; // pitch
//     dx[9] = _ref_data.pitch_dot - o_data.pitch_dot; // pitch_dot

// //----------------------------------------------------------------Leg Length to K------------------------------------------------------------------------------
//     float K[4][10];
//     for(int i = 0; i < P_LOCO_ROW; i++) 
//         for(int j = 0; j < 10; j++) 
//             K[i][j] = p[0][i][j] * o_data.ll * o_data.ll + p[1][i][j] * o_data.ll * o_data.lr + p[2][i][j] * o_data.ll + p[3][i][j] * o_data.lr * o_data.lr + p[4][i][j] * o_data.lr + p[5][i][j];
//     // K is 4x10 matrix 
//     // 4x10 x 10x1 matrix multi (K * dx) 
//     float T_bll = 0;  
//     float T_blr = 0;

//     for(int i = 0; i < 10; i++){
//         _write.torque_wl += K[0][i] * dx[i];
//         _write.torque_wr += K[1][i] * dx[i];
//         T_bll += K[2][i] * dx[i];
//         T_blr += K[3][i] * dx[i];
//     } 

//     // // wheels limit //May not be used
//     // if(output[T_LWL_OUTPUT_NUM] > WHEEL_UPPER_LIMIT) output[T_LWL_OUTPUT_NUM] = WHEEL_UPPER_LIMIT;
//     //     else if(output[T_LWL_OUTPUT_NUM] < WHEEL_LOWER_LIMIT) output[T_LWL_OUTPUT_NUM] = WHEEL_LOWER_LIMIT;
//     // if(output[T_LWR_OUTPUT_NUM] > WHEEL_UPPER_LIMIT) output[T_LWR_OUTPUT_NUM] = WHEEL_UPPER_LIMIT;
//     //     else if(output[T_LWR_OUTPUT_NUM] < WHEEL_LOWER_LIMIT) output[T_LWR_OUTPUT_NUM] = WHEEL_LOWER_LIMIT;



//     //2x2 * 2x1 
//     // jl[a][b]   *   F_bll
//     //   [c][d]       T_bll
//     // = [af + bt]
//     //   [cf + dt]
// //--------------------------------------------------------------Left side force---------------------------------------------------------------------------------
//     if(F_whl >= F_WH_OUTPUT_LIMIT_NUM){
//         _write.torque_fl = o_data.jl[0][0] * F_bll + o_data.jl[0][1] * T_bll;
//         _write.torque_bl = o_data.jl[1][0] * F_bll + o_data.jl[1][1] * T_bll;
//     }else{
//         _write.torque_wl = 0;
//     }
// //-----------------------------------------------------------Right side check force------------------------------------------------------------------------------
//     if(F_whr >= F_WH_OUTPUT_LIMIT_NUM){
//         _write.torque_fr = o_data.jr[0][0] * F_blr + o_data.jr[0][1] * T_blr;
//         _write.torque_br = o_data.jr[1][0] * F_blr + o_data.jr[1][1] * T_blr;
//     }else{
//         _write.torque_wr = 0;
//     }

// //-------------------------------------------------------------Calculate CAN value-----------------------------------------------
//     _write.torque_wl /= -5.0;
//     _write.torque_wr /= 5.0;
//     _write.torque_fl /= -37.0;
//     _write.torque_bl /= -37.0;
//     _write.torque_fr /= 37.0;
//     _write.torque_br /= 37.0;

// return;
// }

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
    // float pitch_dot;
    // float yaw_dot;
    // float yaw_ddot;
    // float b_speed;
    // float b_accel;
    // float ll;
    // float lr;
    // float ll_ddot;
    // float lr_ddot;
    // float theta_ll;
    // float theta_lr;
    // float theta_ll_dot;
    // float theta_lr_dot;
    // float jl[2][2];
    // float jr[2][2];
    Serial.print("pitch_dot: ");
    Serial.println(o_data.pitch_dot*57.29578);
    Serial.print("yaw_dot: ");
    Serial.println(o_data.yaw_dot*57.29578);
    Serial.print("yaw_ddot: ");
    Serial.println(o_data.yaw_ddot*57.29578);
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
    Serial.printf("%E", o_data.ll);
    Serial.println();
    Serial.print("leglength lr: ");
    Serial.printf("%E", o_data.lr);
    Serial.println();
    Serial.print("ll_ddot: ");
    Serial.println(o_data.ll_ddot);
    Serial.print("lr_ddot: ");
    Serial.println(o_data.lr_ddot);
    Serial.print("theta_ll: ");
    Serial.printf("%E", o_data.theta_ll*57.29578);
    Serial.println();
    Serial.print("theta_lr: ");
    Serial.printf("%E", o_data.theta_lr*57.29578);
    Serial.println();
    Serial.print("theta_ll_dot: ");
    Serial.printf("%E", o_data.theta_ll_dot*57.29578);
    Serial.println();
    Serial.print("theta_lr_dot: ");
    Serial.printf("%E", o_data.theta_lr_dot*57.29578);
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

}

void balancing_test::print_visual(){
    return ;
}