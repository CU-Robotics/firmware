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
        {0,0,0,0,0,0,0,0,0,0}, // torque_wl
        {0,0,0,0,0,0,0,0,0,0}, // torque_wr
        {0,0,0,0,0,0,0,0,0,0}, // T_bll
        {0,0,0,0,0,0,0,0,0,0}},// T_blr
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
    // For test purpose
    float tempK[4][10] = {
        {-0.3157,-0.6659,-0.2234,-0.1460,-4.6021,-0.5745,-1.7850,-0.1971,-3.0104,-0.4326}, 
        {-0.3157,-0.6659,0.2234,0.1460,-1.7850,-0.1971,-4.6021,-0.5745,-3.0104,-0.4326}, 
        {0.0557,0.1116,-1.0806,-0.8358,6.7055,0.7714,-7.0740,-0.7067,-222.0533,-7.2328}, 
        {0.0557,0.1116,1.0806,0.8358,-7.0740,-0.7067,6.7055,0.7714,-222.0533,-7.2328}
        };
    memcpy(K,tempK,sizeof(tempK));
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

    o_data.pitch_dot = _data.gyro_pitch > 0.05 || _data.gyro_pitch < -0.05 ? _data.gyro_pitch : 0; 
    // o_data.pitch_dot = (_data.imu_angle_pitch  - o_data.pitch_old) / _dt;        // pitch_dot
    o_data.yaw_dot = _data.gyro_yew > 0.05 || _data.gyro_yew < -0.05 ? _data.gyro_yew : 0;                                                // yaw_dot    
    o_data.yaw_ddot = (o_data.yaw_dot - o_data.yaw_dot_old) / _dt;                   // yaw_ddot 

    o_data.pitch_old = _data.imu_angle_pitch; 
    o_data.yaw_dot_old = o_data.yaw_dot; 
    o_data.b_speed =  (1.0f/2) * (R_w) * (_data.speed_wl + _data.speed_wr); // s_dot //speed
    o_data.s = o_data.b_speed * _dt;
    o_data.b_accel = (o_data.b_speed - o_data.b_speed_old) / _dt; //s_ddot //acceleration


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

    o_data.theta_ll = (fmod((M_PI_2 + _data.imu_angle_pitch - phi0l + M_PI), 2 * M_PI) - M_PI);
    
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

    o_data.theta_lr = (fmod((M_PI_2 + _data.imu_angle_pitch - phi0r + M_PI), (2 * M_PI)) - M_PI);
//-----------------------------------------------------------Get theta_ll_dot and theta_lr_dot-------------------------------------------------------
    o_data.theta_ll_dot = (o_data.theta_ll - o_data.theta_ll_old) / _dt;
    o_data.theta_ll_old = o_data.theta_ll;
    o_data.theta_lr_dot = (o_data.theta_lr - o_data.theta_lr_old) / _dt;
    o_data.theta_lr_old = o_data.theta_lr;

//-----------------------------------------------------------Get ll_ddot and lr_ddot-------------------------------------------------------------------
    float ll_dot = (o_data.ll - o_data.ll_old) / _dt;
    o_data.ll_old = o_data.ll;
    o_data.ll_ddot = (ll_dot - o_data.ll_dot_old) / _dt;
    o_data.ll_dot_old = ll_dot;

    float lr_dot = (o_data.lr - o_data.lr_old) / _dt;
    o_data.lr_old = o_data.lr;
    o_data.lr_ddot = (lr_dot - o_data.lr_dot_old) / _dt;
    o_data.lr_dot_old = lr_dot;
    return;
}


void balancing_test::simple_control(){
// //------------------------------------------------------------leg_controller------------------------------------------------------
//     float l = (o_data.ll + o_data.lr) / 2; // Get the average leg length 
//     float F_psi = pid1.filter(_dt, BOUND, WARP) * (_ref_data.goal_roll - _data.gyro_roll); //Set the PID for roll angle
//     float F_l = pid2.filter(_dt, BOUND, WARP) * (_ref_data.goal_l - l); //Set the PID for leg length 

//     float iF_r = ((((m_b / 2) + (eta_l * m_l)) * l * o_data.pitch_dot * o_data.b_speed) / 2) / R_l; 
//     float iF_l = iF_r; 

// //-----------------------------------------------------------------gravity feed forware-------------------------------------------------------------------
//     float costheta_l = cos(o_data.theta_ll);
//     float costheta_r = cos(o_data.theta_lr);
//     float GF_help = (m_b / 2 + m_l * eta_l) * G_CONSTANT;
//     float gF_l = GF_help * costheta_l; // We calculated the gravity force to left
//     float gF_r = GF_help * costheta_r; // We calculated the gravity force to right

//     float F_bll = F_psi + F_l + iF_l + gF_l;
//     float F_blr = -F_psi + F_l + iF_r + gF_r; 
    
// //---------------------------------------------------------------The NormalF Left------------------------------------------------------------------------
//     float F_whl = F_bll * costheta_l + m_l * (G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.ll_ddot * costheta_l);
// //----------------------------------------------------------------The NormalF Right----------------------------------------------------------------------- 
//     float F_whr = F_blr * costheta_r + m_l * (G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.lr_ddot * costheta_r);


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
    float F_whl = F_bll * costheta_l + m_l * (G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.ll_ddot * costheta_l);
//----------------------------------------------------------------The NormalF Right----------------------------------------------------------------------- 
    float F_whr = F_blr * costheta_r + m_l * (G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.lr_ddot * costheta_r);

    // do nothing check
    if(F_whl < F_WH_OUTPUT_LIMIT_NUM && F_whr < F_WH_OUTPUT_LIMIT_NUM)
        return;

//---------------------------------------------------------------locomotion_controller--------------------------------------------------------------------------
//--------------------------------------------------------------Acceleration Saturation---------------------------------------------------------------------------
    float dx[10];
    // dx[0] = 0; //Ignore // s
    dx[0] = _ref_data.s - o_data.s; // s
    dx[1] = _ref_data.b_speed - o_data.b_speed; // s_dot //speed
    dx[2] = 0; //Ignore // yaw
    //dx[2] = ref[0][2] - obs[0][2]; // yaw angle //We don't have this data and don't need it
    dx[3] = _ref_data.yaw_dot - o_data.yaw_dot; // yaw rotational speed
    dx[4] = _ref_data.theta_ll - o_data.theta_ll; // theta_ll
    dx[5] = _ref_data.theta_ll_dot - o_data.theta_ll_dot; // theta_ll_dot
    dx[6] = _ref_data.theta_lr - o_data.theta_lr; // theta_lr
    dx[7] = _ref_data.theta_lr_dot - o_data.theta_lr_dot; // theta_lr_dot
    dx[8] = _ref_data.pitch - _data.gyro_pitch; // pitch
    dx[9] = _ref_data.pitch_dot - o_data.pitch_dot; // pitch_dot

//----------------------------------------------------------------Leg Length to K------------------------------------------------------------------------------
    // for(int i = 0; i < P_LOCO_ROW; i++) 
    //     for(int j = 0; j < 10; j++) 
    //         K[i][j] = p[0][i][j] * o_data.ll * o_data.ll + p[1][i][j] * o_data.ll * o_data.lr + p[2][i][j] * o_data.ll + p[3][i][j] * o_data.lr * o_data.lr + p[4][i][j] * o_data.lr + p[5][i][j];
    // K is 4x10 matrix 
    // 4x10 x 10x1 matrix multi (K * dx) 
    float T_bll = 0;  
    float T_blr = 0;

    for(int i = 0; i < 10; i++){
        _write.torque_wl += K[0][i] * dx[i];
        _write.torque_wr += K[1][i] * dx[i];
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
    if(F_whl >= F_WH_OUTPUT_LIMIT_NUM){
        _write.torque_fl = o_data.jl[0][0] * F_bll + o_data.jl[0][1] * T_bll;
        _write.torque_bl = o_data.jl[1][0] * F_bll + o_data.jl[1][1] * T_bll;
    }else{
        _write.torque_wl = 0;
    }
//-----------------------------------------------------------Right side check force------------------------------------------------------------------------------
    if(F_whr >= F_WH_OUTPUT_LIMIT_NUM){
        _write.torque_fr = o_data.jr[0][0] * F_blr + o_data.jr[0][1] * T_blr;
        _write.torque_br = o_data.jr[1][0] * F_blr + o_data.jr[1][1] * T_blr;
    }else{
        _write.torque_wr = 0;
    }
//-------------------------------------------------------------Calculate CAN value-----------------------------------------------
    _write.torque_wl /= -5.0;
    _write.torque_wr /= 5.0;

    _write.torque_fl /= -37.0;
    if(_write.torque_fl > MGlimit)
    _write.torque_fl = MGlimit;
    if(_write.torque_fl < -MGlimit)
    _write.torque_fl = -MGlimit;

    _write.torque_bl /= -37.0;
    if(_write.torque_bl > MGlimit)
    _write.torque_bl = MGlimit;
    if(_write.torque_bl < -MGlimit)
    _write.torque_bl = -MGlimit;

    _write.torque_fr /= 37.0;
    if(_write.torque_fr > MGlimit)
    _write.torque_fr = MGlimit;
    if(_write.torque_fr < -MGlimit)
    _write.torque_fr = -MGlimit;

    _write.torque_br /= 37.0;
    if(_write.torque_br > MGlimit)
    _write.torque_br = MGlimit;
    if(_write.torque_br < -MGlimit)
    _write.torque_br = -MGlimit;
return;
}




/// @brief Control part
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
    float F_psi = pid1.filter(_dt, BOUND, WARP) * (_ref_data.goal_roll - _data.gyro_roll); //Set the PID for roll angle
    float F_l = pid2.filter(_dt, BOUND, WARP) * (_ref_data.goal_l - l); //Set the PID for leg length 

    float iF_r = ((((m_b / 2) + (eta_l * m_l)) * l * o_data.pitch_dot * o_data.b_speed) / 2) / R_l;
    float iF_l = -iF_r; 

//-----------------------------------------------------------------gravity force forware-------------------------------------------------------------------
    float costheta_l = cos(o_data.theta_ll);
    float costheta_r = cos(o_data.theta_lr);
    float GF_help = (m_b / 2 + m_l * eta_l) * G_CONSTANT;
    float gF_l = GF_help * costheta_l;
    float gF_r = GF_help * costheta_r; 

    float F_bll = F_psi + F_l + iF_l + gF_l;
    float F_blr = -F_psi + F_l + iF_r + gF_r; 
    
//---------------------------------------------------------------The NormalF Left------------------------------------------------------------------------
    float F_whl = F_bll * costheta_l + m_l * (G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.ll_ddot * costheta_l);
//----------------------------------------------------------------The NormalF Right----------------------------------------------------------------------- 
    float F_whr = F_blr * costheta_r + m_l * (G_CONSTANT + _data.imu_accel_z - (1-eta_l) * o_data.lr_ddot * costheta_r);

    // do nothing check
    if(F_whl < F_WH_OUTPUT_LIMIT_NUM && F_whr < F_WH_OUTPUT_LIMIT_NUM)
        return;

//---------------------------------------------------------------locomotion_controller--------------------------------------------------------------------------
//--------------------------------------------------------------Acceleration Saturation---------------------------------------------------------------------------
    float dx[10];
    dx[0] = 0; //Ignore // s
    //dx[0] = ref[0][0] - obs[0][0]; // s-33A~33A
    dx[1] = _ref_data.b_speed - o_data.b_speed; // s_dot //speed
    dx[2] = 0; //Ignore // yaw
    //dx[2] = ref[0][2] - obs[0][2]; // yaw angle //We don't have this data and don't need it
    dx[3] = _ref_data.yaw_dot - o_data.yaw_dot; // yaw rotational speed
    dx[4] = _ref_data.theta_ll - o_data.theta_ll; // theta_ll
    dx[5] = _ref_data.theta_ll_dot - o_data.theta_ll_dot; // theta_ll_dot
    dx[6] = _ref_data.theta_lr - o_data.theta_lr; // theta_lr
    dx[7] = _ref_data.theta_lr_dot - o_data.theta_lr_dot; // theta_lr_dot
    dx[8] = _ref_data.pitch - _data.gyro_pitch; // pitch
    dx[9] = _ref_data.pitch_dot - o_data.pitch_dot; // pitch_dot

//----------------------------------------------------------------Leg Length to K------------------------------------------------------------------------------
    float K[4][10];
    for(int i = 0; i < P_LOCO_ROW; i++) 
        for(int j = 0; j < 10; j++) 
            K[i][j] = p[0][i][j] * o_data.ll * o_data.ll + p[1][i][j] * o_data.ll * o_data.lr + p[2][i][j] * o_data.ll + p[3][i][j] * o_data.lr * o_data.lr + p[4][i][j] * o_data.lr + p[5][i][j];
    // K is 4x10 matrix 
    // 4x10 x 10x1 matrix multi (K * dx) 
    float T_bll = 0;  
    float T_blr = 0;

    for(int i = 0; i < 10; i++){
        _write.torque_wl += K[0][i] * dx[i];
        _write.torque_wr += K[1][i] * dx[i];
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
    if(F_whl >= F_WH_OUTPUT_LIMIT_NUM){
        _write.torque_fl = o_data.jl[0][0] * F_bll + o_data.jl[0][1] * T_bll;
        _write.torque_bl = o_data.jl[1][0] * F_bll + o_data.jl[1][1] * T_bll;
    }else{
        _write.torque_wl = 0;
    }
//-----------------------------------------------------------Right side check force------------------------------------------------------------------------------
    if(F_whr >= F_WH_OUTPUT_LIMIT_NUM){
        _write.torque_fr = o_data.jr[0][0] * F_blr + o_data.jr[0][1] * T_blr;
        _write.torque_br = o_data.jr[1][0] * F_blr + o_data.jr[1][1] * T_blr;
    }else{
        _write.torque_wr = 0;
    }

//-------------------------------------------------------------Calculate CAN value-----------------------------------------------
    _write.torque_wl /= -5.0;
    _write.torque_wr /= 5.0;
    _write.torque_fl /= -37.0;
    _write.torque_bl /= -37.0;
    _write.torque_fr /= 37.0;
    _write.torque_br /= 37.0;

return;
}

write_data balancing_test::getwrite(){
    return _write;
}

void balancing_test::printdata(){
    Serial.print("fr: ");
    Serial.println(_write.torque_fr);
    Serial.print("fl: ");
    Serial.println(_write.torque_fl);
    Serial.print("bl: ");
    Serial.println(_write.torque_bl);
    Serial.print("br: ");
    Serial.println(_write.torque_br);
    Serial.print("wl: ");
    Serial.println(_write.torque_wl);
    Serial.print("wr: ");
    Serial.println(_write.torque_wr);
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
    Serial.println(o_data.pitch_dot);
    Serial.print("yaw_dot: ");
    Serial.println(o_data.yaw_dot);
    Serial.print("yaw_ddot: ");
    Serial.println(o_data.yaw_ddot);
    Serial.print("b_speed: ");
    Serial.println(o_data.b_speed);
    Serial.print("b_accel: ");
    Serial.println(o_data.b_accel);
    Serial.print("ll: ");
    Serial.printf("%E", o_data.ll);
    Serial.print("lr: ");
    Serial.printf("%E", o_data.lr);
    Serial.print("ll_ddot: ");
    Serial.println(o_data.ll_ddot);
    Serial.print("lr_ddot: ");
    Serial.println(o_data.lr_ddot);
    Serial.print("theta_ll: ");
    Serial.printf("%E", o_data.theta_ll);
    Serial.print("theta_lr: ");
    Serial.printf("%E", o_data.theta_lr);
    Serial.print("theta_ll_dot: ");
    Serial.println(o_data.theta_ll_dot);
    Serial.print("theta_lr_dot: ");
    Serial.println(o_data.theta_lr_dot);
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