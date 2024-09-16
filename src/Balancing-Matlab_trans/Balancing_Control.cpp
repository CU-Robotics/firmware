#include "Balancing_Control.hpp"

BalancingControl::BalancingControl(){}

void BalancingControl::init(){
    pid1.set_K({K1_P, K1_I, K1_D, K1_F});
    pid2.set_K({K2_P, K2_I, K2_D, K2_F});
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

void BalancingControl::step(float output[NUM_MOTORS], float x_d[XHELP_LENGTH], float psi_d, float l_d, float x[XHELP_LENGTH], float psi, float ll, float lr, float jl[4], float jr[4], float a_z){
    float dt = timer.delta(); 

    float l = (ll + lr) / 2; 
    /** This is the part for leg_controller */
        float F_psi = pid1.filter(dt, BOUND, WARP) * (psi_d - psi); //Will comment this 
        float F_l = pid2.filter(dt, BOUND, WARP) * (l_d - l); // Will comment this

        /** In inertia_ff */
            //s_dot = _x[XHELP_s_dot], s_ddot = _x[XHELP_s_ddot], phi_dot = _x[XHELP_phi_dot], phi_ddot = _x[XHELP_phi_ddot], theta_ll = _x[XHELP_theta_ll], theta_lr = _x[XHELP_theta_lr]
            float iffhelp = (m_b / 2 + m_l * eta_l) * (x[XHELP_phi_ddot] * R_l + x[XHELP_s_ddot]);
            float iF_l = -iffhelp * sin(x[XHELP_theta_ll]);
            float iF_r = iffhelp * sin(x[XHELP_theta_lr]); 
            /* F = (m_b / 2 + eta_l * m_l) * l * _x[XHELP_phi_dot]* _x[XHELP_s_dot] / 2 / R_l;
            float Fr = F;
            float Fl = -Fr; */

        /** In gravity_ff */
            float gffhelp = (m_b / 2 + m_l * THE_C_IDK) * G_CONSTANT;
            float gF_l = gffhelp * cos(x[XHELP_theta_ll]);
            float gF_r = gffhelp * cos(x[XHELP_theta_lr]);

        float F_bll = F_psi * MA0 + F_l * MA1 + iF_l * MA2 + iF_r * MA3 + gF_l * MA4 + gF_r * MA5; //ASK for if the matrix will change or not
        float F_blr = F_psi * MA0 + F_l * MA1 + iF_l * MB2 + iF_r * MB3 + gF_l * MB4 + gF_r * MB5;

    /** This is the part for locomotion_controller */ // ASK
        /** In Acceleration Saturation */
            // dx is a array but only used the element 1 in matlab which is 0 here
            float dx[XHELP_LENGTH];
            for(int i = 0; i < 10; i++)
                dx[i] = x_d[i] - x[i];

            if(dx[XHELP_s] < 1 && dx[XHELP_s] > -1)
                dx[XHELP_s] = fabs((dx[XHELP_s]) / (dx[XHELP_s]));
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
        output[T_LWL_OUTPUT_NUM] = constrain(output[T_LWL_OUTPUT_NUM], WHEEL_LOWER_LIMIT, WHEEL_UPPER_LIMIT);// YO!!!!!First output get
        output[T_LWL_OUTPUT_NUM] = constrain(output[T_LWL_OUTPUT_NUM], WHEEL_LOWER_LIMIT, WHEEL_UPPER_LIMIT);// YO!!!!!Second output get
          
            

}   