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
       
    o_data.pitch_dot = (_data.imu_angle_pitch  - o_data.pitch_old) / _dt;        // pitch_dot
    o_data.yaw_dot = _data.gyro_yew;                                                // yaw_dot    
    o_data.yaw_ddot = (o_data.yaw_dot - o_data.yaw_dot_old) / _dt;                   // yaw_ddot 

    o_data.pitch_old = _data.imu_angle_pitch; 
    o_data.yaw_dot_old = o_data.yaw_dot;

    o_data.b_speed =  (1.0f/2) * (R_w) * (_data.speed_wl + _data.speed_wr); // s_dot //speed
    o_data.b_accel = (o_data.b_speed - o_data.b_speed_old) / _dt; //s_ddot //acceleration


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
    float phi2l= 2 * atan2(phihelpl, C0l + A0l);
    //phi2l= fmod(phi2l, 2 * M_PIf); // NO need
    float phi3l= 2 * atan2(phihelpl, C0l - A0l);

    float xCl = l_u * cphi1_l + l_l * cos(phi2l);
    float yCl = l_u * sphi1_l+ l_l * sin(phi2l);

    float helpingl = (xCl - l_a);
    o_data.ll = sqrt(yCl * yCl + helpingl * helpingl); //ll
    float phi0l = atan2(yCl, helpingl); //phi 0

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
    float phi2r= 2 * atan2(phihelpr, c0r + a0r);
    //phi2r= fmod(phi2r, 2 * M_PI); // NO need
    float phi3r= 2 * atan2(phihelpr, c0r - a0r);

    float xCr = l_u * cphi1r + l_l * cos(phi2r);
    float yCr = l_u * sphi1r + l_l * sin(phi2r);

    float helpingr = xCr - l_a;
    o_data.lr = sqrt(yCr * yCr + helpingr * helpingr); // lr
    float phi0r = atan2(yCr, helpingr);

    o_data.jr[0][0]  = (l_u * sin(phi0r - phi3r) * sin(phi1_r - phi2r)) / sin(phi2r - phi3r); 
    o_data.jr[0][0]  = -(l_u * cos(phi0r - phi3r) * sin(phi1_r - phi2r)) / (o_data.lr * sin(phi2r - phi3r));
    o_data.jr[0][0]  = (l_u * sin(phi0r - phi2r) * sin(phi3r - phi4_r)) / sin(phi2r - phi3r);
    o_data.jr[0][0]  = -(l_u * cos(phi0r - phi2r) * sin(phi3r - phi4_r)) / (o_data.lr * sin(phi2r - phi3r));

    o_data.theta_lr = fmod((M_PI_2 + _data.imu_angle_pitch - phi0r + M_PI), (2 * M_PI)) - M_PI;
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
    float F_psi = pid1.filter(_dt, BOUND, WARP) * (ref_data.goal_roll - _data.gyro_roll); //Set the PID for roll angle
    float F_l = pid2.filter(_dt, BOUND, WARP) * (ref_data.goal_l - l); //Set the PID for leg length 

    float iF_r = ((((m_b / 2) + (eta_l * m_l)) * l * o_data.pitch_dot * o_data.b_speed) / 2) / R_l;
    float iF_l = -iF_r; 

    /** In gravity_ff */
    float costheta_l = cos(o_data.theta_ll);
    float costheta_r = cos(o_data.theta_lr);
    float gffhelp = (m_b / 2 + m_l * eta_l) * G_CONSTANT;
    float gF_l = gffhelp * costheta_l;
    float gF_r = gffhelp * costheta_r;//This can be #define
    //Matrix muilti {Checked}
    float F_bll = F_psi + F_l + iF_l + gF_l;
    float F_blr = -F_psi + F_l + iF_r + gF_r; 
    
    /**The NormalF Left */
    float F_whl = F_bll * costheta_l + m_l * (G_CONSTANT + obs[7][2] - (1-eta_l) * obs[8][0] * costheta_l);
        
    /**The NormalF Right */ 
    float F_whr = F_blr * costheta_r + m_l * (G_CONSTANT + obs[7][2] - (1-eta_l) * obs[8][1] * costheta_r);

    if(F_whl < F_WH_OUTPUT_LIMIT_NUM && F_whr < F_WH_OUTPUT_LIMIT_NUM)
        return;

    /** This is the part for locomotion_controller */
    /** In Acceleration Saturation */
    // dx is a array but only used the element 1 in matlab which is 0 here
    float dx[10];
    dx[0] = ref[0][0] - obs[0][0];
    dx[1] = ref[0][1] - obs[0][1];
    dx[2] = ref[0][2] - obs[0][2]; //phi yaw angle
    dx[3] = ref[1][0] - obs[1][0];
    dx[4] = ref[1][1] - obs[1][1];
    dx[5] = ref[1][2] - obs[1][2];
    dx[6] = ref[2][0] - obs[2][0];
    dx[7] = ref[2][1] - obs[2][1];
    dx[8] = ref[2][2] - obs[2][2];
    dx[9] = ref[3][0] - obs[3][0];

        
    if(dx[XHELP_s]>1) dx[XHELP_s] = 1;
    else if(dx[XHELP_s] < -1) dx[XHELP_s] = -1;
    /** In Leg Length to K */
    float K[4][10];
    for(int i = 0; i < P_LOCO_ROW; i++)
        for(int j = 0; j < 10; j++)
            K[i][j] = p[0][i][j] * obs[4][1] * obs[4][1] + p[1][i][j] * obs[4][1] * obs[4][2] + p[2][i][j] * obs[4][1] + p[3][i][j] * obs[4][2] * obs[4][2] + p[4][i][j] * obs[4][2] + p[5][i][j];
    // K = p; // Please change p 
    /* for(int i = 0; i < P_LOCO_ROW; i++) //Please change p
        for(int j = 0; j < 10; j++)
            K = p[0][i][j] * l * l * l + p[1][i][j] * l * l + p[2][i][j] * l + p[3][i][j]; */
    // K is 4x10 matrix
    // 4x10 x 10x1 matrix multi (K * dx)
    float T_bll = 0;  
    float T_blr = 0; 
    for(int i = 0; i < 10; i++){
        output[T_LWL_OUTPUT_NUM] += K[0][i] * dx[i];
        output[T_LWR_OUTPUT_NUM] += K[1][i] * dx[i];
        T_bll += K[2][i] * dx[i];
        T_blr += K[3][i] * dx[i];
    } 
    if(output[T_LWL_OUTPUT_NUM] > WHEEL_UPPER_LIMIT) output[T_LWL_OUTPUT_NUM] = WHEEL_UPPER_LIMIT;
        else if(output[T_LWL_OUTPUT_NUM] < WHEEL_LOWER_LIMIT) output[T_LWL_OUTPUT_NUM] = WHEEL_LOWER_LIMIT;
    if(output[T_LWR_OUTPUT_NUM] > WHEEL_UPPER_LIMIT) output[T_LWR_OUTPUT_NUM] = WHEEL_UPPER_LIMIT;
        else if(output[T_LWR_OUTPUT_NUM] < WHEEL_LOWER_LIMIT) output[T_LWR_OUTPUT_NUM] = WHEEL_LOWER_LIMIT;

    //2x2 * 2x1 
    // jl[a][b]   *   F_bll
    //   [c][d]       T_bll
    // = [af + bt]
    //   [cf + dt]
    //Left side check force
    if(F_whl >= F_WH_OUTPUT_LIMIT_NUM){
        output[T_JLF_OUTPUT_NUM] = obs[5][0] * F_bll + obs[5][1] * T_bll;
        output[T_JLB_OUTPUT_NUM] = obs[5][2] * F_bll + obs[6][0] * F_bll;
    }else{
        output[T_LWL_OUTPUT_NUM] = 0;
        //output[T_JLF_OUTPUT_NUM] = 0;
        //output[T_JLB_OUTPUT_NUM] = 0; //No need for these lines
    }
    //Right side check force
    if(F_whr >= F_WH_OUTPUT_LIMIT_NUM){
        output[T_JRF_OUTPUT_NUM] = obs[6][1] * F_blr + obs[6][2] * T_blr;
        output[T_JRB_OUTPUT_NUM] = obs[7][0] * F_blr + obs[7][1] * T_blr;
    }else{
        output[T_LWR_OUTPUT_NUM] = 0;
        //output[T_JRF_OUTPUT_NUM] = 0;
        //output[T_JRB_OUTPUT_NUM] = 0; //No need for these lines
    }
return;
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