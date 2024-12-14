#include "Balancing_Observer.hpp"

void BalancingObserver::init(){
    _theta_b_old = 0;
    _phi_old = 0;
    _phi_dot_old = 0;
    _theta_ll_old = 0;
    _theta_lr_old = 0;
    _s_dot_old = 0;
    _ll_old = 0;
    _ll_dot_old = 0;
    _lr_old = 0;
    _lr_dot_old = 0;
    // Better run the step 2 times before the while loop to avoid control code calculate something weird. !!!!!!!!!
}

BalancingObserver::BalancingObserver(){}

//for x[12] = [s 0, s_dot 1, phi 2, phi_dot 3, theta_ll 4, theta_ll_dot 5, theta_lr 6, theta_lr_dot 7, theta_b 8, theta_b_dot 9, s_ddot 10, phi_ddot 11] 
//{x[12] = obs[0-11], psi = obs[12], ll = obs[13], lr = obs[14], jl[0][0] = obs[15], jl[0][1] = obs[16], jl[1][0] = obs[17], jl[1][1] = obs[18], jr[0][0] = obs[19], jr[0][1] = obs[20], jr[1][0] = obs[21], jr[1][1] = obs[22], a_z = obs[23], ll_ddot = obs[24], lr_ddot = obs[25]}
//new obs[0][0-2], obs[1][3-5], obs[2][6-8], obs[3][9-11], obs[4][12-14], obs[5][15-17], obs[6][18-20], obs[7][21-23], obs[8][24-26]
void BalancingObserver::step(CANData* can, IMUData* imu, float obs[9][3]){
    // not used but needed
    //a_x is not used in anywhere
    float dt = timer.delta();
    // From sensors
    obs[2][2] = imu->k_pitch;                                   // theta_b    // need check 
    obs[3][0] = (obs[2][2] - _theta_b_old)/ dt;                 // theta_b_dot
    _theta_b_old = obs[2][2];


//** I don't think we need phi */
    //obs[0][2] = imu->gyro_Z;                                    // phi        // need check

    obs[1][0] = imu->gyro_Z;                                    // phi_dot    // need check
    //_phi_old = obs[0][2]; 
    obs[3][2] = (obs[1][0] - _phi_dot_old) / dt;                // phi_ddot  
    _phi_dot_old = obs[1][0];


    obs[4][0] = imu->k_roll;                                    // psi        // need check 

    obs[7][2] = imu->accel_Z;                                   // a_z        // need check 

    
    // float theta_wl = can->get_motor_attribute(L_CAN, L_W_MOTORID, ANGLE); // we don't need this
    float theta_wl_dot = can->get_motor_attribute(L_CAN, L_W_MOTORID, SPEED);
    // float theta_wr = can->get_motor_attribute(L_CAN, L_W_MOTORID, ANGLE); // we don't need this
    float theta_wr_dot = can->get_motor_attribute(R_CAN, R_W_MOTORID, SPEED);


    float phi4_l = can->get_motor_attribute(L_CAN, L_FJ_MOTORID, ANGLE);                // need check
    float phi1_l = can->get_motor_attribute(L_CAN, L_BJ_MOTORID, ANGLE);                // need check
    float phi4_r = can->get_motor_attribute(R_CAN, R_FJ_MOTORID, ANGLE);                // need check
    float phi1_r = can->get_motor_attribute(R_CAN, R_BJ_MOTORID, ANGLE);                // need check


    // obs[0][0] =  (1.0f/2) * (R_w) * (theta_wl + theta_wr); // s // we don't need this 
    obs[0][1] =  (1.0f/2) * (R_w) * (theta_wl_dot + theta_wr_dot); // s_dot

    obs[3][1] = (obs[0][1] - _s_dot_old) / dt;

    // Left Leg Forward Kinematics & Jacobian
    // theta_ll = obs[4], theta_ll_dot = obs[5], ll = obs[13] ,jl[0][0] = obs[15], jl[0][1] = obs[16], jl[1][0] = obs[17], jl[1][1] = obs[18]
    
    // phi4_r = M_PIf - phi4_r;
    // phi1_r = M_PIf - phi1_r; // Need testing

    float cphi1_l = cos(phi1_l);
    float sphi1_l = sin(phi1_l);
    float cphi4_l = cos(phi4_l);
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
    float yCl = l_u * sphi1_l+l_l * sin(phi2l);
    //float xC_l = (l_u * cphi4_l + l_l * cos(phi3l) + 2 * l_a);

    float helpingl = (xCl - l_a);
    obs[4][1] = sqrt(yCl * yCl + helpingl * helpingl);//ll
    float phi0l = atan2(yCl, helpingl);

    obs[5][0] = (l_u * sin(phi0l - phi3l) * sin(phi1_l - phi2l)) / sin(phi2l - phi3l); 
    obs[5][1] = -(l_u * cos(phi0l - phi3l) * sin(phi1_l - phi2l)) / (obs[4][1] * sin(phi2l - phi3l));
    obs[5][2] = (l_u * sin(phi0l - phi2l) * sin(phi3l - phi4_l)) / sin(phi2l - phi3l);
    obs[6][0] = -(l_u * cos(phi0l - phi2l) * sin(phi3l - phi4_l)) / (obs[4][1] * sin(phi2l - phi3l));

    obs[1][1] = fmod((M_PI_2 + obs[2][2] - phi0l + M_PI), 2 * M_PI) - M_PI;
    
 
    // Right Leg Forward Kinematics & Jacobian

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
    float phihelpr = sqrt(b0r * b0r + a0r * a0r - c0r * c0r) - b0r;
    float phi2r= 2 * atan2(phihelpr, a0r + c0r);
    phi2r= fmod(phi2r, 2 * M_PI);
    float phi3r= 2 * atan2(phihelpr, c0r - a0r);

    float xCr = l_u * cphi1r + l_l * cos(phi2r);
    float yCr = l_u * sphi1r + l_l * sin(phi2r);
    //float xC_r = (l_u * cphi4r + l_l * cos(phi3r) + 2 * l_a);

    float helpingr = xCr - l_a;
    obs[4][2] = sqrt(yCr * yCr + helpingr * helpingr); // lr
    float phi0r = atan2(yCr, helpingr);

    obs[6][1] = (l_u * sin(phi0r - phi3r) * sin(phi1_r - phi2r)) / sin(phi2r - phi3r); 
    obs[6][2] = -(l_u * cos(phi0r - phi3r) * sin(phi1_r - phi2r)) / (obs[4][2] * sin(phi2r - phi3r));
    obs[7][0] = (l_u * sin(phi0r - phi2r) * sin(phi3r - phi4_r)) / sin(phi2r - phi3r);
    obs[7][1] = -(l_u * cos(phi0r - phi2r) * sin(phi3r - phi4_r)) / (obs[4][2] * sin(phi2r - phi3r));

    obs[2][0] = fmod((M_PI_2 + obs[2][2] - phi0r + M_PI), (2 * M_PI)) - M_PI;

    //get theta_ll_dot and theta_lr_dot
    obs[1][2] = (obs[1][1] - _theta_ll_old) / dt;
    _theta_ll_old = obs[1][1];
    obs[2][1] = (obs[2][0] - _theta_lr_old) / dt;
    _theta_lr_old = obs[2][0];

    // get ll_ddot and lr_ddot
    float ll_dot = (obs[4][1] - _ll_old) / dt;
    _ll_old = ll_dot;
    obs[8][0] = (ll_dot - _ll_dot_old) / dt;
    _ll_dot_old = ll_dot;

    float lr_dot = (obs[4][1] - _lr_old) / dt;
    _lr_old = lr_dot;
    obs[8][1] = (lr_dot - _lr_dot_old) / dt;
    _lr_dot_old = lr_dot;
    return;
}

void BalancingObserver::testprint(float obs[9][3]){
   
    Serial.print("\ns = ");
    Serial.print(obs[0][0]);
    Serial.print("\ns_dot = ");
    Serial.print(obs[0][1]);
    Serial.print("\nphi = ");
    Serial.print(obs[0][2]);
    Serial.print("\nphi_dot = ");
    Serial.print(obs[1][0]);
    Serial.print("\ntheta_ll = ");
    Serial.print(obs[1][1]);
    Serial.print("\ntheta_ll_dot  = ");
    Serial.print(obs[1][2]);
    Serial.print("\ntheta_lr = ");
    Serial.print(obs[2][0]);
    Serial.print("\ntheta_lr_dot = ");
    Serial.print(obs[2][1]);
    Serial.print("\ntheta_b = ");
    Serial.print(obs[2][2]);
    Serial.print("\ntheta_b_dot = ");
    Serial.print(obs[3][0]);
    Serial.print("\ns_ddot = ");
    Serial.print(obs[3][1]);
    Serial.print("\nphi_ddot = ");
    Serial.print(obs[3][2]);
    Serial.print("\npsi = ");
    Serial.print(obs[4][0]);
    Serial.print("\nll = ");
    Serial.print(obs[4][1]);
    Serial.print("\nlr = ");
    Serial.print(obs[4][2]);


    Serial.print("\njl = [");
    Serial.print(obs[5][0]);
    Serial.print("] [");
    Serial.print(obs[5][1]);
    Serial.print("]");
    Serial.print("\n[");
    Serial.print(obs[5][2]);
    Serial.print("] [");
    Serial.print(obs[6][0]);
    Serial.print("]");

    Serial.print("\njr = [");
    Serial.print(obs[6][1]);
    Serial.print("] [");
    Serial.print(obs[6][2]);
    Serial.print("]");
    Serial.print("\n[");
    Serial.print(obs[7][0]);
    Serial.print("] [");
    Serial.print(obs[7][1]);
    Serial.print("]");
    
    Serial.print("\na_z = ");
    Serial.print(obs[7][2]);
    Serial.print("\nll_ddot = ");
    Serial.print(obs[8][0]);
    Serial.print("\nlr_ddot = ");
    Serial.print(obs[8][1]);
}

void BalancingObserver::settingprint(CANData *can, IMUData *imu){
    Serial.print("\n ----------New update line----------");
    Serial.print("\nL_W_MOTOR: ");
    Serial.print("\nANGLE = ");
    Serial.print(can->get_motor_attribute(L_CAN, L_W_MOTORID, ANGLE));
    Serial.print("\nSPEED = ");
    Serial.print(can->get_motor_attribute(L_CAN, L_W_MOTORID, SPEED));
    Serial.print("\nTORQUE = ");
    Serial.print(can->get_motor_attribute(L_CAN, L_W_MOTORID, TORQUE));


    Serial.print("\nL_FJ_MOTOR: ");
    Serial.print("\nANGLE = ");
    Serial.print(can->get_motor_attribute(L_CAN, L_FJ_MOTORID, ANGLE));
    Serial.print("\nSPEED = ");
    Serial.print(can->get_motor_attribute(L_CAN, L_FJ_MOTORID, SPEED));
    Serial.print("\nTORQUE = ");
    Serial.print(can->get_motor_attribute(L_CAN, L_FJ_MOTORID, TORQUE));
    
    Serial.print("\nL_BJ_MOTOR: ");
    Serial.print("\nANGLE = ");
    Serial.print(can->get_motor_attribute(L_CAN, L_BJ_MOTORID, ANGLE));
    Serial.print("\nSPEED = ");
    Serial.print(can->get_motor_attribute(L_CAN, L_BJ_MOTORID, SPEED));
    Serial.print("\nTORQUE = ");
    Serial.print(can->get_motor_attribute(L_CAN, L_BJ_MOTORID, TORQUE));
    
    Serial.print("\nR_W_MOTOR: ");
    Serial.print("\nANGLE = ");
    Serial.print(can->get_motor_attribute(R_CAN, R_W_MOTORID, ANGLE));
    Serial.print("\nSPEED = ");
    Serial.print(can->get_motor_attribute(R_CAN, R_W_MOTORID, SPEED));
    Serial.print("\nTORQUE = ");
    Serial.print(can->get_motor_attribute(R_CAN, R_W_MOTORID, TORQUE));


    Serial.print("\nR_FJ_MOTOR: ");
    Serial.print("\nANGLE = ");
    Serial.print(can->get_motor_attribute(R_CAN, R_FJ_MOTORID, ANGLE));
    Serial.print("\nSPEED = ");
    Serial.print(can->get_motor_attribute(R_CAN, R_FJ_MOTORID, SPEED));
    Serial.print("\nTORQUE = ");
    Serial.print(can->get_motor_attribute(R_CAN, R_FJ_MOTORID, TORQUE));
    
    Serial.print("\nR_BJ_MOTOR: ");
    Serial.print("\nANGLE = ");
    Serial.print(can->get_motor_attribute(R_CAN, R_BJ_MOTORID, ANGLE));
    Serial.print("\nSPEED = ");
    Serial.print(can->get_motor_attribute(R_CAN, R_BJ_MOTORID, SPEED));
    Serial.print("\nTORQUE = ");
    Serial.print(can->get_motor_attribute(R_CAN, R_BJ_MOTORID, TORQUE));
    
    Serial.print("\nimu acceleration: ");
    Serial.print("\na_X = ");
    Serial.print(imu->accel_X);
    Serial.print("\na_Y = ");
    Serial.print(imu->accel_Y);
    Serial.print("\na_Z = ");
    Serial.print(imu->accel_Z);
    Serial.print("\nimu gyro: ");
    Serial.print("\ngyro_X = ");
    Serial.print(imu->gyro_X);
    Serial.print("\ngyro_Y = ");
    Serial.print(imu->gyro_Y);
    Serial.print("\ngyro_Z = ");
    Serial.print(imu->gyro_Z);
}
