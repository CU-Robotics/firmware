#include "Balancing_Control.hpp"

BalancingControl::BalancingControl(){}

void BalancingControl::init(){
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
/// @param ref array from ref {x_d[12] = ref[0-11], psi_d = ref[12], l_d = ref[13]}
/// @param obs array from observer {x[12] = obs[0-11], psi = obs[12], ll = obs[13], lr = obs[14], jl[0][0] = obs[15], jl[0][1] = obs[16], jl[1][1] = obs[17], jl[1][1] = obs[18], jr[0][0] = obs[19], jr[0][1] = obs[20], jr[1][0] = obs[21], jr[1][1] = obs[22], a_z = obs[23], ll_ddot = obs[24], lr_ddot = obs[25]}
void BalancingControl::step(float output[NUM_MOTORS], float ref[13], float obs[26]){

    float dt = timer.delta(); 
    float l = (obs[13] + obs[14]) / 2; 

    /** Initialize all output variables */
    output[T_LWL_OUTPUT_NUM] = 0;
    output[T_JLF_OUTPUT_NUM] = 0;
    output[T_JLB_OUTPUT_NUM] = 0;
    output[T_LWR_OUTPUT_NUM] = 0;
    output[T_JRF_OUTPUT_NUM] = 0;
    output[T_JRB_OUTPUT_NUM] = 0;

    /** This is the part for leg_controller */
    float F_psi = pid1.filter(dt, BOUND, WARP) * (ref[12] - obs[12]); 
    float F_l = pid2.filter(dt, BOUND, WARP) * (ref[13]- l); 

    /** In inertia_ff */
    //s_dot = _x[XHELP_s_dot], s_ddot = _x[XHELP_s_ddot], phi_dot = _x[XHELP_phi_dot], phi_ddot = _x[XHELP_phi_ddot], theta_ll = _x[XHELP_theta_ll], theta_lr = _x[XHELP_theta_lr]
    float iffhelp = (m_b / 2 + m_l * eta_l) * (obs[XHELP_phi_ddot] * R_l + obs[XHELP_s_ddot]);
    /*float iF_l = -iffhelp * sin(x[XHELP_theta_ll]);
    float iF_r = iffhelp * sin(x[XHELP_theta_lr]); 
    F =
    */

    float iF_r = (m_b / 2 + eta_l * m_l) * l * obs[XHELP_phi_dot] * obs[XHELP_s_dot] / 2 / R_l;
    float iF_l = -iF_r; 

    /** In gravity_ff */
    float costheta_l = cos(obs[XHELP_theta_ll]);
    float costheta_r = cos(obs[XHELP_theta_lr]);
    float gffhelp = (m_b / 2 + m_l * eta_l) * G_CONSTANT;
    float gF_l = gffhelp * costheta_l;
    float gF_r = gffhelp * costheta_r;//This can be #define
    //Matrix muilti {Checked}
    float F_bll = F_psi + F_l + iF_l + gF_l;
    float F_blr = -F_psi + F_l + iF_r + gF_r; 
    
    /**The NormalF Left */
    float F_whl = F_bll * costheta_l + m_l * (G_CONSTANT + obs[23] - (1-eta_l) * obs[24] * costheta_l);
        
    /**The NormalF Right */ 
    float F_whr = F_blr * costheta_r + m_l * (G_CONSTANT + obs[23] - (1-eta_l) * obs[25] * costheta_r);

    if(F_whl < F_WH_OUTPUT_LIMIT_NUM && F_whr < F_WH_OUTPUT_LIMIT_NUM)
        return;

    /** This is the part for locomotion_controller */
    /** In Acceleration Saturation */
    // dx is a array but only used the element 1 in matlab which is 0 here
    float dx[XHELP_LENGTH];
    for(int i = 0; i < 10; i++)
        dx[i] = ref[i] - ref[i];
    if(dx[XHELP_s]>1) dx[XHELP_s] = 1;
    else if(dx[XHELP_s] < -1) dx[XHELP_s] = -1;
    /** In Leg Length to K */
    float K[4][10];
    for(int i = 0; i < P_LOCO_ROW; i++)
        for(int j = 0; j < 10; j++)
            K[i][j] = p[0][i][j] * obs[13] * obs[13] + p[1][i][j] * obs[13] * obs[14] + p[2][i][j] * obs[13] + p[3][i][j] * obs[14] * obs[14] + p[4][i][j] * obs[14] + p[5][i][j];
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
        output[T_JLF_OUTPUT_NUM] = obs[15] * F_bll + obs[16] * T_bll;
        output[T_JLB_OUTPUT_NUM] = obs[17] * F_bll + obs[18] * F_bll;
    }else{
        output[T_LWL_OUTPUT_NUM] = 0;
        //output[T_JLF_OUTPUT_NUM] = 0;
        //output[T_JLB_OUTPUT_NUM] = 0; //No need for these lines
    }
    //Right side check force
    if(F_whr >= F_WH_OUTPUT_LIMIT_NUM){
        output[T_JRF_OUTPUT_NUM] = obs[19] * F_blr + obs[20] * T_blr;
        output[T_JRB_OUTPUT_NUM] = obs[21] * F_blr + obs[22] * T_blr;
    }else{
        output[T_LWR_OUTPUT_NUM] = 0;
        //output[T_JRF_OUTPUT_NUM] = 0;
        //output[T_JRB_OUTPUT_NUM] = 0; //No need for these lines
    }
return;
}   