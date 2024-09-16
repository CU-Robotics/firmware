#include "Balancing_Control.hpp"

BalancingControl::BalancingControl(){}

void BalancingControl::init(){
    float gain1[4] = {K1_P, K1_I, K1_D, K1_F};
    float gain2[4] = {K2_P, K2_I, K2_D, K2_F};
    pid1.set_K(gain1);
    pid2.set_K(gain2);
    // p[rows = P_LOCO_ROW] [cols = XHELP_LENGTH]
    float tempp[6][P_LOCO_ROW][10] =
        {{{1,0,0,0,0,0,0,0,0,0},  
        {0,2,0,0,0,0,0,0,0,0}, 
        {0,0,3,0,0,0,0,0,0,0}, 
        {0,0,0,4,0,0,0,0,0,0}},
        {{1,0,0,0,0,0,0,0,0,0},  
        {0,2,0,0,0,0,0,0,0,0}, 
        {0,0,3,0,0,0,0,0,0,0}, 
        {0,0,0,4,0,0,0,0,0,0}},
        {{1,0,0,0,0,0,0,0,0,0},  
        {0,2,0,0,0,0,0,0,0,0}, 
        {0,0,3,0,0,0,0,0,0,0}, 
        {0,0,0,4,0,0,0,0,0,0}},
        {{1,0,0,0,0,0,0,0,0,0},  
        {0,2,0,0,0,0,0,0,0,0}, 
        {0,0,3,0,0,0,0,0,0,0}, 
        {0,0,0,4,0,0,0,0,0,0}},
        {{1,0,0,0,0,0,0,0,0,0},  
        {0,2,0,0,0,0,0,0,0,0}, 
        {0,0,3,0,0,0,0,0,0,0}, 
        {0,0,0,4,0,0,0,0,0,0}},
        {{1,0,0,0,0,0,0,0,0,0},  
        {0,2,0,0,0,0,0,0,0,0}, 
        {0,0,3,0,0,0,0,0,0,0}, 
        {0,0,0,4,0,0,0,0,0,0}}};
        
    memcpy(p,tempp,sizeof(tempp));
}

void BalancingControl::step(float output[NUM_MOTORS], float x_d[XHELP_LENGTH], float psi_d, float l_d, float x[XHELP_LENGTH], float psi, float ll, float lr, float jl[2][2], float jr[2][2], float a_z, float ll_ddot, float lr_ddot){
    float dt = timer.delta(); 
    float l = (ll + lr) / 2; 

    /** This is the part for leg_controller */
    float F_psi = pid1.filter(dt, BOUND, WARP) * (psi_d - psi); 
    float F_l = pid2.filter(dt, BOUND, WARP) * (l_d - l); 

    /** In inertia_ff */
    //s_dot = _x[XHELP_s_dot], s_ddot = _x[XHELP_s_ddot], phi_dot = _x[XHELP_phi_dot], phi_ddot = _x[XHELP_phi_ddot], theta_ll = _x[XHELP_theta_ll], theta_lr = _x[XHELP_theta_lr]
    float iffhelp = (m_b / 2 + m_l * eta_l) * (x[XHELP_phi_ddot] * R_l + x[XHELP_s_ddot]);
    float iF_l = -iffhelp * sin(x[XHELP_theta_ll]);
    float iF_r = iffhelp * sin(x[XHELP_theta_lr]); 
    /* F = (m_b / 2 + eta_l * m_l) * l * _x[XHELP_phi_dot]* _x[XHELP_s_dot] / 2 / R_l;
    float Fr = F;
    float Fl = -Fr; */

    /** In gravity_ff */
    float costheta_l = cos(x[XHELP_theta_ll]);
    float costheta_r = cos(x[XHELP_theta_lr]);
    float gffhelp = (m_b / 2 + m_l * THE_C_IDK) * G_CONSTANT;
    float gF_l = gffhelp * costheta_l;
    float gF_r = gffhelp * costheta_r;
    //Matrix muilti
    float F_bll = F_psi * MA0 + F_l * MA1 + iF_l * MA2 + iF_r * MA3 + gF_l * MA4 + gF_r * MA5; 
    float F_blr = F_psi * MA0 + F_l * MA1 + iF_l * MB2 + iF_r * MB3 + gF_l * MB4 + gF_r * MB5;
    
    /**The NormalF Left */
    float F_whl = F_bll * costheta_l + m_l * (G_CONSTANT + a_z - (1-eta_l) * ll_ddot * costheta_l);
        
    /**The NormalF Right */ //ASK
    float F_whr = F_blr * costheta_r + m_l * (G_CONSTANT + a_z - (1-eta_l) * lr_ddot * costheta_r);

    if(F_whl < F_WH_OUTPUT_LIMIT_NUM && F_whr < F_WH_OUTPUT_LIMIT_NUM){
        output[T_LWL_OUTPUT_NUM] = 0;
        output[T_JLF_OUTPUT_NUM] = 0;
        output[T_JLB_OUTPUT_NUM] = 0;
        output[T_LWR_OUTPUT_NUM] = 0;
        output[T_JRF_OUTPUT_NUM] = 0;
        output[T_JRB_OUTPUT_NUM] = 0;
        return;
    }

    /** This is the part for locomotion_controller */
    /** In Acceleration Saturation */
    // dx is a array but only used the element 1 in matlab which is 0 here
    float dx[XHELP_LENGTH];
    for(int i = 0; i < 10; i++)
        dx[i] = x_d[i] - x[i];
    if(dx[XHELP_s]>1) dx[XHELP_s] = 1; // constrain is like 2 times slower
    else if(dx[XHELP_s] < -1) dx[XHELP_s] = -1;
    /** In Leg Length to K */
    float K[4][10];
    for(int i = 0; i < P_LOCO_ROW; i++)
        for(int j = 0; j < 10; j++)
            K[i][j] = p[0][i][j] * ll * ll + p[1][i][j] * ll * lr + p[2][i][j] * ll + p[3][i][j] * lr * lr + p[4][i][j] * lr + p[5][i][j];
    // K = p; // Please change p 
    /* for(int i = 0; i < P_LOCO_ROW; i++) //Please change p
        for(int j = 0; j < 10; j++)
            K = p[0][i][j] * l * l * l + p[1][i][j] * l * l + p[2][i][j] * l + p[3][i][j]; */
    // Fortunatly we get K 4x10
    // 4x10 x 10x1 matrix multi (K * dx)
    float T_bll;  
    float T_blr; 
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
    // = [af + bg]
    //   [cf + dg]
    //Left side check force
    if(F_whl >= F_WH_OUTPUT_LIMIT_NUM){
        output[T_JLF_OUTPUT_NUM] = jl[0][0] * F_bll + jl[0][1] * F_bll;
        output[T_JLB_OUTPUT_NUM] = jl[1][0] * F_bll + jl[1][1] * F_bll;
    }else{
        output[T_LWL_OUTPUT_NUM] = 0;
        output[T_JLF_OUTPUT_NUM] = 0;
        output[T_JLB_OUTPUT_NUM] = 0;
    }
    //Right side check force
    if(F_whr >= F_WH_OUTPUT_LIMIT_NUM){
        output[T_JRF_OUTPUT_NUM] = jr[0][0] * F_blr + jr[0][1] * F_blr;
        output[T_JRB_OUTPUT_NUM] = jr[1][0] * F_blr + jr[1][1] * F_blr;
    }else{
        output[T_LWR_OUTPUT_NUM] = 0;
        output[T_JRF_OUTPUT_NUM] = 0;
        output[T_JRB_OUTPUT_NUM] = 0;
    }
return;
}   