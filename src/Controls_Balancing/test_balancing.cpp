#include "test_balancing.hpp"

void balancing_test::init(){
    slowdalay_help = micros();
    o_data.Q = 0.6; // Need tune
    o_data.R_v = 0.00001; // Need tune
    o_data.R_a = 0.1; // Need tune
    o_data.P[0][0] = 10;
    o_data.P[0][1] = 10;
    o_data.P[1][0] = 10;
    o_data.P[1][1] = 10;
    o_data.K[0][0] = 0;
    o_data.K[0][1] = 0;
    o_data.K[1][0] = 0;
    o_data.K[1][1] = 0;
    //PID1:roll, PID2:Leg length
    float gain1[4] = {K1_P, K1_I, K1_D, K1_F};
    float gain2[4] = {K2_P, K2_I, K2_D, K2_F};
    pid1.set_K(gain1);
    pid2.set_K(gain2);

    // p matrix for average leg length
    float tempp[P_LOCO_ROW][4][10] = {
        {
            {-0.345131, -0.820201, -2.012599, -1.028943, 6.551991, 5.169364, -28.131742, 3.441288, -0.486574, 13.661905, },
            {-0.345131, -0.820201, 2.012599, 1.028943, -28.131742, 3.441288, 6.551991, 5.169364, -0.486574, 13.661905, },
            {-4.437724, -23.495997, -21.546051, -11.410433, 43.546815, -2.877318, -73.642383, -2.393106, -25.862558, -10.791666, },
            {-4.437724, -23.495997, 21.546051, 11.410433, -73.642383, -2.393106, 43.546815, -2.877318, -25.862558, -10.791666, },
        },
        {
            {0.318252, 0.809070, 0.182998, 0.115914, 0.354209, -5.243803, 22.469987, -3.385791, -1.698062, -13.421109, },
            {0.318252, 0.809070, -0.182998, -0.115914, 22.469987, -3.385791, 0.354209, -5.243803, -1.698062, -13.421109, },
            {4.283221, 22.568197, 24.460118, 12.882622, -68.038588, 1.165928, 94.211057, 2.694054, 24.858781, 9.883646, },
            {4.283221, 22.568197, -24.460118, -12.882622, 94.211057, 2.694054, -68.038588, 1.165928, 24.858781, 9.883646, },
        },
        {
            {-0.108510, -0.434782, 1.523810, 0.828435, -12.958869, -0.344395, -3.423864, 0.292643, 2.725719, 5.198045, },
            {-0.108510, -0.434782, -1.523810, -0.828435, -3.423864, 0.292643, -12.958869, -0.344395, 2.725719, 5.198045, },
            {-1.639377, -8.566081, -9.580798, -5.120682, 38.512534, 1.459998, -46.814778, -1.678512, -9.068379, -3.278287, },
            {-1.639377, -8.566081, 9.580798, 5.120682, -46.814778, -1.678512, 38.512534, 1.459998, -9.068379, -3.278287, },
        },
        {
            {-0.298065, -1.618429, -0.506192, -0.239358, -2.641439, -0.375880, -2.790641, -0.342435, -1.805698, -1.029699, },
            {-0.298065, -1.618429, 0.506192, 0.239358, -2.790641, -0.342435, -2.641439, -0.375880, -1.805698, -1.029699, },
            {0.376976, 1.984374, -0.366737, -0.208761, 1.985193, 0.433753, 0.560780, -0.132260, -46.845666, -6.579072, },
            {0.376976, 1.984374, 0.366737, 0.208761, 0.560780, -0.132260, 1.985193, 0.433753, -46.845666, -6.579072, },
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
    _ref_data.goal_l = 0.25;
    _ref_data.goal_roll = 0;
    _ref_data.s = 0; 
    _ref_data.s_dot = 0;
    _ref_data.pitch = 0;
    _ref_data.pitch_dot = 0;
    _ref_data.theta_ll = 0;
    _ref_data.theta_ll_dot = 0;
    _ref_data.theta_lr = 0;
    _ref_data.theta_lr_dot = 0;
    _ref_data.yaw_dot = 0;

    o_data.control_yaw = 0;
    o_data.body_speed_filtered = 0;
    o_data.body_accel_filtered = 0;
    o_data.wheel_speed_old = 0;
    o_data.gyro_yaw_old = 0;
    o_data.wheel_speed_dot = 0;
    o_data.gyro_yaw_dot = 0;
    o_data.control_s = 0;
    o_data.b_speed = 0;
    o_data.b_accel = 0;
    o_data.s_dot_filtered = 0;
    o_data.ll = 0;
    o_data.lr = 0;
    o_data.ll_ddot = 0;
    o_data.lr_ddot = 0;
    o_data.theta_ll = 0;
    o_data.theta_lr = 0;
    o_data.ll_dot = 0;
    o_data.lr_dot = 0;
    o_data.theta_ll_dot = 0;
    o_data.theta_lr_dot = 0;
    o_data.avg_count = 0;
    o_data.jl[0][0] = 0;
    o_data.jl[0][1] = 0;
    o_data.jl[1][0] = 0;
    o_data.jl[1][1] = 0;
    o_data.jr[0][0] = 0;
    o_data.jr[0][1] = 0;
    o_data.jr[1][0] = 0;
    o_data.jr[1][1] = 0;
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
    o_data.control_yaw += _data.gyro_yaw * _dt;

    o_data.gyro_yaw_dot = (_data.gyro_yaw - o_data.gyro_yaw_old) / _dt;
    o_data.gyro_yaw_old = _data.gyro_yaw;
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
    o_data.jl[0][1] = -(l_u * cos(phi0l - phi3l) * sin(phi1_l - phi2l)) / (o_data.ll * sin(phi2l - phi3l));
    o_data.jl[1][0] = (l_u * sin(phi0l - phi2l) * sin(phi3l - phi4_l)) / sin(phi2l - phi3l);
    o_data.jl[1][1] = -(l_u * cos(phi0l - phi2l) * sin(phi3l - phi4_l)) / (o_data.ll * sin(phi2l - phi3l));

    o_data.theta_ll = -(fmod((M_PI_2 - phi0l - _data.imu_angle_pitch), 2 * M_PI)); // This is the correct one 
    if(o_data.theta_ll < -M_PI){ // less than -180 degree
        o_data.theta_ll = (2*M_PI) - o_data.theta_ll; // change it into positive angle
    }
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
    o_data.jr[0][1]  = -(l_u * cos(phi0r - phi3r) * sin(phi1_r - phi2r)) / (o_data.ll * sin(phi2r - phi3r));
    o_data.jr[1][0]  = (l_u * sin(phi0r - phi2r) * sin(phi3r - phi4_r)) / sin(phi2r - phi3r);
    o_data.jr[1][1]  = -(l_u * cos(phi0r - phi2r) * sin(phi3r - phi4_r)) / (o_data.ll *sin(phi2r - phi3r));

    o_data.theta_lr = -(fmod((M_PI_2 - phi0r - _data.imu_angle_pitch), (2 * M_PI)));
    if(o_data.theta_lr < -M_PI){ // less than -180 degree
        o_data.theta_lr = (2*M_PI) - o_data.theta_lr; // change it into positive angle
    }

//----------------------------------------------------Calculate Theta_of_leg_dot for both leg--------------------------------------
    float phi2_dot_l = (l_u *((_data.speed_bl * sphi1_l - _data.speed_fl * sphi4_l) * cos(phi3l) + ((-_data.speed_bl) * cphi1_l + _data.speed_fl * cphi4_l) * sin(phi3l))) / (l_l * sin(phi2l - phi3l));
    float xC_dot_l = -(l_u * (-_data.speed_bl) * sphi1_l + l_l * phi2_dot_l * sin(phi2l));
    float yC_dot_l = (l_u * (-_data.speed_bl) * cphi1_l + l_l * phi2_dot_l * cos(phi2l));
    o_data.ll_dot = ((helpingl*xC_dot_l + yCl*yC_dot_l)/o_data.ll);
    o_data.theta_ll_dot = (((helpingl*yC_dot_l) - (yCl * xC_dot_l)) / (helpingl * helpingl + yCl * yCl)) - _data.gyro_pitch;


    float phi2_dot_r = (l_u * ((-_data.speed_br * sphi1r + _data.speed_fr * sphi4r) * cos(phi3r) + (_data.speed_br * cphi1r - _data.speed_fr * cphi4r) * sin(phi3r))) / (l_l * sin(phi2r - phi3r));
    float xC_dot_r = -(l_u * _data.speed_br * sphi1r + l_l * phi2_dot_r * sin(phi2r));
    float yC_dot_r = (l_u * _data.speed_br * cphi1r + l_l * phi2_dot_r * cos(phi2r));
    o_data.lr_dot = ((helpingr*xC_dot_r + yCr*yC_dot_r)/o_data.lr);
    o_data.theta_lr_dot = (((helpingr*yC_dot_r) - (yCr * xC_dot_r)) / (helpingr * helpingr + yCr * yCr)) - _data.gyro_pitch;
//--------------------------------------------------------------b_s and filter for it--------------------------------------------------------
    

    // o_data.s_dot_filtered =  1/2 * 0.05 * (_data.speed_wr - _data.speed_wl) ; // s_dot //speed 
    // o_data.s_dot_filtered =  0.05/2 * (-_data.speed_wl + _data.speed_wr) ;

    

    // o_data.wheel_speed_dot = (o_data.s_dot_filtered - o_data.wheel_speed_old) / _dt;
    // o_data.wheel_speed_old = o_data.s_dot_filtered;
    o_data.s_dot_unfiltered = - 0.05/2 * (_data.speed_wl + _data.speed_wr);
    o_data.b_speed = o_data.s_dot_unfiltered - (1/2) * (o_data.ll*(o_data.theta_ll_dot + _data.imu_angle_pitch)*cos(phi0l) + o_data.lr*(o_data.theta_lr_dot + _data.imu_angle_pitch)*cos(phi0r)) - (1/2)* (o_data.ll_dot * sin(phi0l) + o_data.lr_dot * sin(phi0r));
    o_data.b_accel = _data.imu_accel_x;
       
//-------------------------------------------filter by a kalman filter (I will update this soon) ---------------------------------------------------------------------
    // For the x we have [v,a]^T 
    // predict
    o_data.body_speed_filtered += o_data.body_accel_filtered * _dt;

    float P_hat[2][2] = {0};
    float P_FP[2][2] = {0};
    float F[2][2] = {{1,_dt},{0,1}};
    float Q[2][2] = {{0.25f*_dt*_dt*_dt*_dt, 0.5f*_dt*_dt*_dt},{0.5f*_dt*_dt*_dt, _dt*_dt}};
    // P_hat = F * P * F^T + Q
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                P_FP[i][j] += F[i][k] * o_data.P[k][j];
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                P_hat[i][j] += P_FP[i][k] * F[j][k];
            }
            P_hat[i][j] += Q[i][j] * o_data.Q * o_data.Q;
        }
    }
    
    // Update
    float error_speed = o_data.b_speed - o_data.body_speed_filtered;
    float error_accel = o_data.b_accel - o_data.body_accel_filtered;
    float S[2][2] = {0};
    for(int i =0; i < 2; i++){
        for(int j = 0; j < 2; j++){
            S[i][j] = P_hat[i][j];
        }
    }
    // if(abs(o_data.s_dot_unfiltered)< 0.01){ // Since if the wheel is really slow, there shouldn't be any slip possible.
    //     S[0][0] += 0.000001 * 0.000001;
    // }else{
    //     S[0][0] += o_data.R_v * o_data.R_v;
    // }

    S[0][0] += o_data.R_v * o_data.R_v;
    S[1][1] += o_data.R_a * o_data.R_a;
    float S_inv[2][2] = {0};
    float detS = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    S_inv[0][0] = S[1][1] / detS;
    S_inv[1][1] = S[0][0] / detS;
    S_inv[0][1] = -S[0][1] / detS;
    S_inv[1][0] = -S[1][0] / detS;
    float K[2][2] = {0};
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 2; j++){
            for(int k = 0; k < 2; k++){
                K[i][j] += P_hat[i][k] * S_inv[k][j];
            }
        }
    }
    o_data.body_speed_filtered += K[0][0] * error_speed;
    o_data.body_accel_filtered += K[1][1] * error_accel;
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 2; j++){
            if(i==j)
            K[i][j] = (1 - K[i][j]);
            else 
            K[i][j] = -K[i][j];
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                o_data.P[i][j] = K[i][k] * P_hat[j][k];
            }
        }
    }
    

    o_data.s_dot_filtered = o_data.body_speed_filtered + (1/2) * (o_data.ll*(o_data.theta_ll_dot + _data.imu_angle_pitch)*cos(phi0l) + o_data.lr*(o_data.theta_lr_dot + _data.imu_angle_pitch)*cos(phi0r)) - (1/2)* (o_data.ll_dot * sin(phi0l) + o_data.lr_dot * sin(phi0r));
    o_data.wheel_speed_dot = (o_data.s_dot_filtered - o_data.wheel_speed_old) / _dt;
    o_data.wheel_speed_old = o_data.s_dot_filtered;
    if(abs(o_data.s_dot_filtered) < 0.01){ //0.1cm/s
        o_data.s_dot_filtered = 0;
    }
    
    o_data.control_s += o_data.s_dot_filtered * _dt; 

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
    // float F_psi = 1 * pid1.filter(_dt, NOBOUND, WARP, _ref_data.goal_roll, _data.imu_angle_roll, 0.4); //Set the PID for roll angle
    float F_psi = 1 * pid1.filter(_dt, NOBOUND, WARP, _ref_data.goal_roll, _data.imu_angle_roll, 0.9, true, _data.gyro_roll); //Set the PID for roll angle using sensor derivative data.
    // float F_l = 1 * pid2.filter(_dt, NOBOUND, NOWARP, _ref_data.goal_l, l, 0.9); //Set the PID for leg length 
    float F_l = 1 * pid2.filter(_dt, NOBOUND, NOWARP, _ref_data.goal_l, l, 0.9, true, ((o_data.lr_dot + o_data.ll_dot) * 0.5f)); //Set the PID for leg length using encoder data

    // float iF_r = ((((m_b / 2) + (eta_l * m_l)) * l * _data.gyro_pitch * o_data.s_dot_filtered) / 2) / R_l;
    // float iF_l = -iF_r; 

    //Inertial Feedforward (IF) calculation
    float iF_l = ((m_b / 2) + (eta_l * m_l))*(o_data.gyro_yaw_dot * R_l + o_data.wheel_speed_dot) * sin(o_data.theta_ll);
    float iF_r = ((m_b / 2) + (eta_l * m_l))*(-(o_data.gyro_yaw_dot * R_l) + o_data.wheel_speed_dot) * sin(o_data.theta_lr);
//-----------------------------------------------------------------gravity feed forware-------------------------------------------------------------------
    float costheta_l = cos(o_data.theta_ll);
    float costheta_r = cos(o_data.theta_lr);

    float GF_help = (m_b / 2 + m_l * eta_l) * G_CONSTANT;
    float gF_l = GF_help * costheta_l;
    float gF_r = GF_help * costheta_r; 
    float F_bll = F_psi + F_l + iF_l + gF_l;
    float F_blr = -F_psi + F_l + iF_r + gF_r; 

    _debug_data.F_psi = F_psi;

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
    dx[0] = _ref_data.s - o_data.control_s; // s
    dx[1] = _ref_data.s_dot - o_data.s_dot_filtered; // speed
    // dx[2] = 0; //Ignore // yaw
    dx[2] = _ref_data.yaw -o_data.control_yaw; // yaw angle //We don't have this data and don't need it
    if(dx[2] < -180 * DEG_TO_RAD)
        dx[2] += 360 * DEG_TO_RAD;
    dx[3] = _ref_data.yaw_dot - _data.gyro_yaw; // yaw rotational speed in deg
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
    
    int wheel_motor__left_sign = -1;
    int wheel_motor__right_sign = 1;
    int leg_motor__left_sign = -1;
    int leg_motor__right_sign = 1;


    if(_write.torque_wl > WHEEL_MOTOR_limit)
    _write.torque_wl = WHEEL_MOTOR_limit;
    if(_write.torque_wl < -WHEEL_MOTOR_limit)
    _write.torque_wl = -WHEEL_MOTOR_limit;
    _write.torque_wl /= wheel_motor__left_sign * 5.0;
    
    if(_write.torque_wr > WHEEL_MOTOR_limit)
    _write.torque_wr = WHEEL_MOTOR_limit;
    if(_write.torque_wr < -WHEEL_MOTOR_limit)
    _write.torque_wr = -WHEEL_MOTOR_limit;
    _write.torque_wr /= wheel_motor__right_sign * 5.0;
    
    if(_write.torque_fl > MGlimit)
    _write.torque_fl = MGlimit;
    if(_write.torque_fl < -MGlimit)
    _write.torque_fl = -MGlimit;
    _write.torque_fl /= leg_motor__left_sign * 37.0;

    if(_write.torque_bl > MGlimit)
    _write.torque_bl = MGlimit;
    if(_write.torque_bl < -MGlimit)
    _write.torque_bl = -MGlimit;
    _write.torque_bl /= leg_motor__left_sign * 37.0;

    if(_write.torque_fr > MGlimit)
    _write.torque_fr = MGlimit;
    if(_write.torque_fr < -MGlimit)
    _write.torque_fr = -MGlimit;
    _write.torque_fr /= leg_motor__right_sign * 37.0;
    
    if(_write.torque_br > MGlimit)
    _write.torque_br = MGlimit;
    if(_write.torque_br < -MGlimit)
    _write.torque_br = -MGlimit;
    _write.torque_br /= leg_motor__right_sign * 37.0;


//----------------------------Save data to printout for debug-------------------------------------------------------------
    _debug_data.F_blr = F_blr;
    _debug_data.F_bll = F_bll;
    _debug_data.T_blr = T_blr;
    _debug_data.T_bll = T_bll;
    _debug_data.F_psi = F_psi;
    return;
}

void balancing_test::reset_s(){
    o_data.control_s = 0;
}

void balancing_test::reset_yaw(){
    o_data.control_yaw = 0;
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

ref_data* balancing_test::get_ref(){
    return &_ref_data;
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
    // Serial.print("speed_wl ");
    // Serial.println(_data.speed_wl);
    // Serial.print("speed_wr ");
    // Serial.println(_data.speed_wr);
    // Serial.print("s: ");
    // Serial.println(o_data.control_s);
    // Serial.print("s_dot_filtered: ");
    // Serial.println(o_data.s_dot_filtered);
    // Serial.print("imu_s: ");
    // Serial.println(o_data.imu_s);
    // Serial.print("imu_speed: ");
    // Serial.println(o_data.imu_speed_x);
    // Serial.print("b_accel: ");
    // Serial.println(o_data.b_accel);
    // Serial.print("leglength ll: ");
    // Serial.printf("%f", o_data.ll);
    // Serial.println();
    // Serial.print("leglength lr: ");
    // Serial.printf("%f", o_data.lr);
    // Serial.println();
    // Serial.print("leglength_dot ll: ");
    // Serial.printf("%f\n", o_data.ll_dot);
    // Serial.print("leglength_dot lr: ");
    // Serial.printf("%f\n", o_data.lr_dot);
    // Serial.print("ll_ddot: ");
    // Serial.println(o_data.ll_ddot);
    // Serial.print("lr_ddot: ");
    // Serial.println(o_data.lr_ddot);
    // Serial.print("theta_ll: ");
    // Serial.printf("%f", o_data.theta_ll*RAD_TO_DEG);
    // Serial.println();
    // Serial.print("theta_lr: ");
    // Serial.printf("%f", o_data.theta_lr*RAD_TO_DEG);
    // Serial.println();
    // Serial.print("theta_ll_dot: ");
    // Serial.printf("%f", o_data.theta_ll_dot*RAD_TO_DEG);
    // Serial.println();
    // Serial.print("theta_lr_dot: ");
    // Serial.printf("%f", o_data.theta_lr_dot*RAD_TO_DEG);
    // Serial.println();
    // Serial.print("jl: ");
    // Serial.print(o_data.jl[0][0]);
    // Serial.print(" ");
    // Serial.println(o_data.jl[0][1]);
    // Serial.print(o_data.jl[1][0]);
    // Serial.print(" ");
    // Serial.println(o_data.jl[1][1]);
    // Serial.print("jr: ");
    // Serial.print(o_data.jr[0][0]);
    // Serial.print(" ");
    // Serial.println(o_data.jr[0][1]);
    // Serial.print(o_data.jr[1][0]);
    // Serial.print(" ");
    // Serial.println(o_data.jr[1][1]);
    Serial.printf("pitch: %f, roll: %f, yaw: %f\n", _data.imu_angle_pitch * RAD_TO_DEG, _data.imu_angle_roll * RAD_TO_DEG, _data.imu_angle_yaw * RAD_TO_DEG);
    Serial.printf("gyro pitch: %f, roll: %f, yaw: %f\n", _data.gyro_pitch * RAD_TO_DEG, _data.gyro_roll * RAD_TO_DEG, _data.gyro_yaw * RAD_TO_DEG);
}

void balancing_test::print_visual(){
    // Serial.printf("waggle graph %s %f \n", "theta_lr", o_data.theta_lr * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "theta_ll", o_data.theta_ll * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "theta_lr_dot", o_data.theta_lr_dot * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "theta_ll_dot", o_data.theta_ll_dot * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "Pitch", _data.imu_angle_pitch * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "Roll", _data.imu_angle_roll * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "Yaw", _data.imu_angle_yaw * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "Gyro_Pitch", _data.gyro_pitch * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "Gyro_Roll", _data.gyro_roll * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "Gyro_Yaw", _data.gyro_yaw * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "Control_Yaw", o_data.control_yaw * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "F_legl", _debug_data.F_bll);
    // Serial.printf("waggle graph %s %f \n", "F_legr", _debug_data.F_blr);
    // Serial.printf("waggle graph %s %f \n", "T_legl", _debug_data.T_bll);
    // Serial.printf("waggle graph %s %f \n", "T_legr", _debug_data.T_blr);
    // Serial.printf("waggle graph %s %f \n", "F_roll", _debug_data.F_psi);
    Serial.printf("waggle graph %s %f \n", "o_data.b_speed", o_data.b_speed); // Speed without filter
    Serial.printf("waggle graph %s %f \n", "o_data.b_accel", o_data.b_accel); // Acceleration without filter

    Serial.printf("waggle graph %s %f \n", "s_dot_unfiltered", o_data.s_dot_unfiltered); // Speed with filter
    Serial.printf("waggle graph %s %f \n", "s_dot_filtered", o_data.s_dot_filtered); // Speed with filter
    Serial.printf("waggle graph %s %f \n", "control_s", o_data.control_s); // Speed without filter
    Serial.printf("waggle graph %s %f \n", "body_accel_filtered", o_data.body_accel_filtered); // Acceleration with filter


    // Serial.printf("waggle graph %s %f \n", "ref_s", _ref_data.s );
    // Serial.printf("waggle graph %s %f \n", "ref_s_dot", _ref_data.speed);
    // Serial.printf("waggle graph %s %f \n", "ref_yaw", _ref_data.yaw);
    // Serial.printf("waggle graph %s %f \n", "ref_yaw_dot", _ref_data.yaw_dot );
    // Serial.printf("waggle graph %s %f \n", "ref_llength", _ref_data.goal_l);
    
    // Serial.printf("waggle graph %s %f \n", "pitch", _data.imu_angle_pitch * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "roll", _data.imu_angle_roll * RAD_TO_DEG);
    // Serial.printf("waggle graph %s %f \n", "yaw", _data.imu_angle_yaw * RAD_TO_DEG);
    return ;
}