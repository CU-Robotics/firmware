/*mag calibration data*/
static float mx_centre;
static float my_centre;
static float mz_centre;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

float32_t gravity[3];
float32_t MagConst[3];
float32_t gyroBias[3];

uint8_t counter = 0;

float sMag[3] = {0.0f,0.0f,0.0f};
float sAcc[3] = {0.0f,0.0f,0.0f};
float sGyro[3] = {0.0f,0.0f,0.0f};

/*for Kalman Filter*/
arm_matrix_instance_f32 Matrix_X;//state
arm_matrix_instance_f32 Matrix_X_prev; // prev_state
arm_matrix_instance_f32 Matrix_A; //state_transition matrix
arm_matrix_instance_f32 Matrix_Q; //process noise covariacne matrix
arm_matrix_instance_f32 Matrix_P; //state noise covariacne
arm_matrix_instance_f32 Matrix_P_prev; //previous/updated state noise covariance
arm_matrix_instance_f32 Matrix_ra; // accel sensor nosie covariance
arm_matrix_instance_f32 Matrix_rm; // mag sesnor noise covariance
arm_matrix_instance_f32 Matrix_rg; //gyro sensor noise covariance
arm_matrix_instance_f32 Matrix_R; //meas noise covariance
arm_matrix_instance_f32 Matrix_S; 
arm_matrix_instance_f32 Matrix_H; //meas model 
arm_matrix_instance_f32 Matrix_H_T; //meas model transpose
arm_matrix_instance_f32 Matrix_K; //kalman gain
arm_matrix_instance_f32 Matrix_I; // identity matrix
arm_matrix_instance_f32 Matrix_skewX; 
arm_matrix_instance_f32 Matrix_skewX_T;
arm_matrix_instance_f32 Matrix_temp_12; //temporary matrix
arm_matrix_instance_f32 Matrix_temp_12_1;
arm_matrix_instance_f32 Matrix_temp_16;
arm_matrix_instance_f32 Matrix_temp_16_1;
arm_matrix_instance_f32 Matrix_temp_16_T;
arm_matrix_instance_f32 Matrix_temp_32;
arm_matrix_instance_f32 Matrix_temp_32_T;
arm_matrix_instance_f32 Matrix_temp_64;
float32_t accel[3];
float32_t mag[3];
float32_t temp_vector[3];
float32_t data_matrix_X[4] ={1,0,0,0};
float32_t data_matrix_X_prev[4] ={1,0,0,0};
float32_t data_matrix_A[16];
float32_t data_matrix_Q[16];
float32_t data_matrix_P[16];
float32_t data_matrix_P_prev[16] = {1,0,0,0,
                                    0,1,0,0,
                                    0,0,1,0,
                                    0,0,0,1};
float32_t data_matrix_ra[9] = {0.004,0,0,   //0.008 original
                               0,0.004,0,
                               0,0,0.004};
float32_t data_matrix_rm[9] = {0.06,0,0,    //was 0.05
                               0,0.06,0,
                               0,0,0.06};
float32_t data_matrix_rg[9] = {0.01,0,0,
                               0,0.01,0,
                               0,0,0.01};
float32_t data_matrix_R[64] = {0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0};
float32_t data_matrix_S[64];
float32_t data_matrix_H[32];
float32_t data_H_T[32];
float32_t data_matrix_K[32];
float32_t data_matrix_I[16] = {1,0,0,0,
                               0,1,0,0,
                               0,0,1,0,
                               0,0,0,1};
float32_t data_matrix_skewX[12];
float32_t data_skewX_T[12];
float32_t temp_matrix_9[9];
float32_t temp_matrix_12[12];
float32_t temp_matrix_12_1[12];
float32_t temp_matrix_16[16];
float32_t temp_matrix_16_1[16];
float32_t temp_matrix_16_T[16];
float32_t temp_matrix_32[32];
float32_t temp_matrix_32_T[32];
float32_t temp_matrix_64[64];
float32_t temp_value;
float32_t X_norm;
float32_t magnitude;

float32_t dt = 0.002;
uint8_t firstRun = 1; //first run flag;

void runKalmanFilter(){
      /*step 1-> calculate A ->state transition matrix*/
      /*temp1 = [gyro(1)*pi/180;gyro(2)*pi/180;gyro(3)*pi/180];*/
      for (int i=0;i<3;i++){
        temp_vector[i] = Gxyz[i]*3.1415926/180;
      }
      /*magnitude = norm(temp1);*/
      temp_value = temp_vector[0]*temp_vector[0] + temp_vector[1]*temp_vector[1] + temp_vector[2]*temp_vector[2];
      arm_sqrt_f32(temp_value, &magnitude);
  
      /*  if (magnitude<1e-4)
         magnitude = 0;
         temp1 = zeros(3,1);
      else
         temp1 = temp1/magnitude*sin(magnitude*dt/2);*/
      if(magnitude<0.0001){
        magnitude = 0;
        for(int i=0;i<3;i++){
          temp_vector[i] = 0;
        }
      }else{
        float32_t tmp = arm_sin_f32(magnitude*dt/2);
        for(int i=0;i<3;i++){
          temp_vector[i] = temp_vector[i]/magnitude*tmp;//not tested
        }
      }
      /*a = cos(magnitude/2*dt);*/
      temp_value = arm_cos_f32(magnitude*dt/2);
  
      /*skew = skewSymmetric(a,temp1);*/
      /*function [Matrix]=skewSymmetric(a,X)
        Matrix = [a X(3)*(-1) X(2);X(3) a X(1)*(-1);X(2)*(-1) X(1) a];*/
      temp_matrix_9[0] = temp_value;
      temp_matrix_9[1] = temp_vector[2]*(-1);
      temp_matrix_9[2] = temp_vector[1];
      temp_matrix_9[3] = temp_vector[2];
      temp_matrix_9[4] = temp_value;
      temp_matrix_9[5] = temp_vector[0]*(-1);
      temp_matrix_9[6] = temp_vector[1]*(-1);
      temp_matrix_9[7] = temp_vector[0];
      temp_matrix_9[8] = temp_value;
  
      /*value of A*/
      /*  A_top = [a,(temp1')*(-1)];
          A_btm = [temp1,skew];
          A = [A_top;A_btm];
      */
      data_matrix_A[0] = temp_value;
      data_matrix_A[1] = temp_vector[0]*(-1);
      data_matrix_A[2] = temp_vector[1]*(-1);
      data_matrix_A[3] = temp_vector[2]*(-1);
      data_matrix_A[4] = temp_vector[0];
      data_matrix_A[8] = temp_vector[1];
      data_matrix_A[12] = temp_vector[2];
      data_matrix_A[5] = temp_matrix_9[0];
      data_matrix_A[6] = temp_matrix_9[1];
      data_matrix_A[7] = temp_matrix_9[2];
      data_matrix_A[9] = temp_matrix_9[3];
      data_matrix_A[10] = temp_matrix_9[4];
      data_matrix_A[11] = temp_matrix_9[5];
      data_matrix_A[13] = temp_matrix_9[6];
      data_matrix_A[14] = temp_matrix_9[7];
      data_matrix_A[15] = temp_matrix_9[8];
      
      /*Step 2 ---------------->  predict X */
      /*X = A*X_prev;*/
      arm_mat_mult_f32(&Matrix_A,&Matrix_X_prev,&Matrix_X);
  
      /*Step 3 ----------------> calculate Q*/
      /*skewQ = skewSymmetric(X(1),X(2:4));*/
      temp_matrix_9[0] = data_matrix_X[0];
      temp_matrix_9[1] = data_matrix_X[3]*(-1);
      temp_matrix_9[2] = data_matrix_X[2];
      temp_matrix_9[3] = data_matrix_X[3];
      temp_matrix_9[4] = data_matrix_X[0];
      temp_matrix_9[5] = data_matrix_X[1]*(-1);
      temp_matrix_9[6] = data_matrix_X[2]*(-1);
      temp_matrix_9[7] = data_matrix_X[1];
      temp_matrix_9[8] = data_matrix_X[0];
  
      /*skewX = [X(2)*(-1) X(3)*(-1) X(4)*(-1);skewQ];*/
      data_matrix_skewX[0] = data_matrix_X[1]*(-1);
      data_matrix_skewX[1] = data_matrix_X[2]*(-1);
      data_matrix_skewX[2] = data_matrix_X[3]*(-1);
      data_matrix_skewX[3] = temp_matrix_9[0];
      data_matrix_skewX[4] = temp_matrix_9[1];
      data_matrix_skewX[5] = temp_matrix_9[2];
      data_matrix_skewX[6] = temp_matrix_9[3];
      data_matrix_skewX[7] = temp_matrix_9[4];
      data_matrix_skewX[8] = temp_matrix_9[5];
      data_matrix_skewX[9] = temp_matrix_9[6];
      data_matrix_skewX[10] = temp_matrix_9[7];
      data_matrix_skewX[11] = temp_matrix_9[8];
  
      /*Q = dt*dt/4*skewX*rg*(skewX');*/
      temp_value = dt*dt/4;
      arm_mat_mult_f32(&Matrix_skewX,&Matrix_rg,&Matrix_temp_12_1);
      arm_mat_trans_f32(&Matrix_skewX,&Matrix_skewX_T);
      arm_mat_mult_f32(&Matrix_temp_12_1,&Matrix_skewX_T,&Matrix_Q);
      for(int i=0;i<16;i++){
        data_matrix_Q[i] *= temp_value;
      }
  
      /*Step4 ----------------> predict P */ 
      /*P = A*P_prev*A'+ Q;*/
      if(firstRun){
        temp_matrix_16_1[0] = 1;
        temp_matrix_16_1[5] = 1;
        temp_matrix_16_1[10] = 1;
        temp_matrix_16_1[15] = 1;
      }else{
        arm_mat_trans_f32(&Matrix_A,&Matrix_temp_16_T);
        arm_mat_mult_f32(&Matrix_A,&Matrix_P_prev,&Matrix_temp_16);
        arm_mat_mult_f32(&Matrix_temp_16,&Matrix_temp_16_T,&Matrix_temp_16_1);
      }
      arm_mat_add_f32(&Matrix_temp_16_1,&Matrix_Q,&Matrix_P);
  
      /*Step 5 --------------> Calculate H*/
      /*tmp = Za-G;*/
      for(int i=0;i<3;i++){
        temp_vector[i] = Axyz[i]-gravity[i];
      }
  
      /*Hleft = [0;tmp]; + Hright = [(-1)*tmp';skewH*(-1)];*/
      data_matrix_H[0] = 0;
      data_matrix_H[1] = temp_vector[0]*(-1);
      data_matrix_H[2] = temp_vector[1]*(-1);
      data_matrix_H[3] = temp_vector[2]*(-1);
      data_matrix_H[4] = temp_vector[0];
      data_matrix_H[8] = temp_vector[1];
      data_matrix_H[12] = temp_vector[2];
  
      /*tmp1 = Za+G;*/
      for(int i=0;i<3;i++){
        temp_vector[i] = Axyz[i]+gravity[i];
      }
  
      /*skewH = skewSymmetric(0,tmp1);*/
      temp_matrix_9[0] = 0;
      temp_matrix_9[1] = temp_vector[2]*(-1);
      temp_matrix_9[2] = temp_vector[1];
      temp_matrix_9[3] = temp_vector[2];
      temp_matrix_9[4] = 0;
      temp_matrix_9[5] = temp_vector[0]*(-1);
      temp_matrix_9[6] = temp_vector[1]*(-1);
      temp_matrix_9[7] = temp_vector[0];
      temp_matrix_9[8] = 0;
  
      /*Hright = [(-1)*tmp';skewH*(-1)];*/
      /*Htop = [Hleft,Hright];*/
      data_matrix_H[5] = temp_matrix_9[0]*(-1);
      data_matrix_H[6] = temp_matrix_9[1]*(-1);
      data_matrix_H[7] = temp_matrix_9[2]*(-1);
      data_matrix_H[9] = temp_matrix_9[3]*(-1);
      data_matrix_H[10] = temp_matrix_9[4]*(-1);
      data_matrix_H[11] = temp_matrix_9[5]*(-1);
      data_matrix_H[13] = temp_matrix_9[6]*(-1);
      data_matrix_H[14] = temp_matrix_9[7]*(-1);
      data_matrix_H[15] = temp_matrix_9[8]*(-1);
  
      /*tmp = Zm-M;*/
      for(int i=0;i<3;i++){
        temp_vector[i] = Mxyz[i]-MagConst[i];
      }
  
      /*Hleft = [0;tmp]; + Hright = [(-1)*tmp';skewH*(-1)];*/
      data_matrix_H[16] = 0;
      data_matrix_H[17] = temp_vector[0]*(-1);
      data_matrix_H[18] = temp_vector[1]*(-1);
      data_matrix_H[19] = temp_vector[2]*(-1);
      data_matrix_H[20] = temp_vector[0];
      data_matrix_H[24] = temp_vector[1];
      data_matrix_H[28] = temp_vector[2];
  
      /*tmp1 = Zm+M;*/
      for(int i=0;i<3;i++){
        temp_vector[i] = Mxyz[i]+MagConst[i];
      }
  
      /*skewH = skewSymmetric(0,tmp1);*/
      temp_matrix_9[0] = 0;
      temp_matrix_9[1] = temp_vector[2]*(-1);
      temp_matrix_9[2] = temp_vector[1];
      temp_matrix_9[3] = temp_vector[2];
      temp_matrix_9[4] = 0;
      temp_matrix_9[5] = temp_vector[0]*(-1);
      temp_matrix_9[6] = temp_vector[1]*(-1);
      temp_matrix_9[7] = temp_vector[0];
      temp_matrix_9[8] = 0;
  
      /*Hright = [(-1)*tmp';skewH*(-1)];
      Hbtm = [Hleft,Hright];
      H = [Htop;Hbtm];*/
      data_matrix_H[21] = temp_matrix_9[0]*(-1);
      data_matrix_H[22] = temp_matrix_9[1]*(-1);
      data_matrix_H[23] = temp_matrix_9[2]*(-1);
      data_matrix_H[25] = temp_matrix_9[3]*(-1);
      data_matrix_H[26] = temp_matrix_9[4]*(-1);
      data_matrix_H[27] = temp_matrix_9[5]*(-1);
      data_matrix_H[29] = temp_matrix_9[6]*(-1);
      data_matrix_H[30] = temp_matrix_9[7]*(-1);
      data_matrix_H[31] = temp_matrix_9[8]*(-1);
  
      /*Step 6 --------------> Calculate R*/
      /*Ra = 0.25*skewX*ra*(skewX');*/
      arm_mat_mult_f32(&Matrix_skewX,&Matrix_ra,&Matrix_temp_12_1);
      arm_mat_trans_f32(&Matrix_skewX,&Matrix_skewX_T);
      arm_mat_mult_f32(&Matrix_temp_12_1,&Matrix_skewX_T,&Matrix_temp_16);
      for(int i=0;i<16;i++){
        temp_matrix_16[i] *= 0.25;
      }
  
      /*Rm = 0.25*skewX*rm*(skewX');*/
      arm_mat_mult_f32(&Matrix_skewX,&Matrix_rm,&Matrix_temp_12_1);
      arm_mat_trans_f32(&Matrix_skewX,&Matrix_skewX_T);
      arm_mat_mult_f32(&Matrix_temp_12_1,&Matrix_skewX_T,&Matrix_temp_16_1);
      for(int i=0;i<16;i++){
        temp_matrix_16_1[i] *= 0.25;
      }
  
      /*Rtop = [Ra,zeros(4)];
      Rbtm = [zeros(4),Rm];
      R = [Rtop;Rbtm];*/
      data_matrix_R[0] = temp_matrix_16[0];
      data_matrix_R[1] = temp_matrix_16[1];
      data_matrix_R[2] = temp_matrix_16[2];
      data_matrix_R[3] = temp_matrix_16[3];
      data_matrix_R[8] = temp_matrix_16[4];
      data_matrix_R[9] = temp_matrix_16[5];
      data_matrix_R[10] = temp_matrix_16[6];
      data_matrix_R[11] = temp_matrix_16[7];
      data_matrix_R[16] = temp_matrix_16[8];
      data_matrix_R[17] = temp_matrix_16[9];
      data_matrix_R[18] = temp_matrix_16[10];
      data_matrix_R[19] = temp_matrix_16[11];
      data_matrix_R[24] = temp_matrix_16[12];
      data_matrix_R[25] = temp_matrix_16[13];
      data_matrix_R[26] = temp_matrix_16[14];
      data_matrix_R[27] = temp_matrix_16[15];
  
      data_matrix_R[36] = temp_matrix_16_1[0];
      data_matrix_R[37] = temp_matrix_16_1[1];
      data_matrix_R[38] = temp_matrix_16_1[2];
      data_matrix_R[39] = temp_matrix_16_1[3];
      data_matrix_R[44] = temp_matrix_16_1[4];
      data_matrix_R[45] = temp_matrix_16_1[5];
      data_matrix_R[46] = temp_matrix_16_1[6];
      data_matrix_R[47] = temp_matrix_16_1[7];
      data_matrix_R[52] = temp_matrix_16_1[8];
      data_matrix_R[53] = temp_matrix_16_1[9];
      data_matrix_R[54] = temp_matrix_16_1[10];
      data_matrix_R[55] = temp_matrix_16_1[11];
      data_matrix_R[60] = temp_matrix_16_1[12];
      data_matrix_R[61] = temp_matrix_16_1[13];
      data_matrix_R[62] = temp_matrix_16_1[14];
      data_matrix_R[63] = temp_matrix_16_1[15];
  
      //the other values should be 0 as initialized.
  
      /*Step 7 -------------->  update */
      /*S = H*P*H'+R;*/
      arm_mat_trans_f32(&Matrix_H,&Matrix_H_T);
      arm_mat_mult_f32(&Matrix_H,&Matrix_P,&Matrix_temp_32);
      arm_mat_mult_f32(&Matrix_temp_32,&Matrix_H_T,&Matrix_temp_64);
      if (firstRun){
        temp_matrix_64[3] = 0;
        temp_matrix_64[10] = 0;
        temp_matrix_64[17] = 0;
        temp_matrix_64[24] = 0;
        temp_matrix_64[39] = 0;
        temp_matrix_64[46] = 0;
        temp_matrix_64[53] = 0;
        temp_matrix_64[60] = 0;
      }
      arm_mat_add_f32(&Matrix_temp_64,&Matrix_R,&Matrix_S);
  
      /*K = (P*H')/S;*/  
      arm_mat_inverse_f32(&Matrix_S,&Matrix_temp_64);
      arm_mat_mult_f32(&Matrix_P,&Matrix_H_T,&Matrix_temp_32_T);
      arm_mat_mult_f32(&Matrix_temp_32_T,&Matrix_temp_64,&Matrix_K);
  
      /*X_updated = (I-K*H)*X;*/
      arm_mat_mult_f32(&Matrix_K,&Matrix_H,&Matrix_temp_16);
      arm_mat_sub_f32(&Matrix_I,&Matrix_temp_16,&Matrix_temp_16_1);
      arm_mat_mult_f32(&Matrix_temp_16_1,&Matrix_X,&Matrix_X_prev);
  
      /*X_updated = X_updated/norm(X_updated);*/
      temp_value = data_matrix_X_prev[0]*data_matrix_X_prev[0] + data_matrix_X_prev[1]*data_matrix_X_prev[1] + data_matrix_X_prev[2]*data_matrix_X_prev[2] 
               + data_matrix_X_prev[3]*data_matrix_X_prev[3];
      arm_sqrt_f32(temp_value, &X_norm);
      for (int i=0;i<4;i++){
        data_matrix_X_prev[i] = data_matrix_X_prev[i]/X_norm;
      }
  
      /*P_updated = (I-K*H)*P;*/
  
      arm_mat_mult_f32(&Matrix_temp_16_1,&Matrix_P,&Matrix_P_prev);
  
      /*arm_mat_trans_f32(&Matrix_temp_16_1,&Matrix_temp_16_T);
      arm_mat_mult_f32(&Matrix_temp_16_1,&Matrix_P,&Matrix_temp_16);
      arm_mat_mult_f32(&Matrix_temp_16,&Matrix_temp_16_T,&Matrix_temp_16_1);
      arm_mat_trans_f32(&Matrix_K,&Matrix_temp_32);
      arm_mat_mult_f32(&Matrix_K,&Matrix_R,&Matrix_temp_32_T);
      arm_mat_mult_f32(&Matrix_temp_32_T,&Matrix_temp_32,&Matrix_temp_16);
      arm_mat_add_f32(&Matrix_temp_16_1,&Matrix_temp_16,&Matrix_P_prev);*/
      
      if(firstRun){
        firstRun = 0;
      }
}

void initMatrix(){
  arm_mat_init_f32(&Matrix_X,4,1,(float32_t*)data_matrix_X);
  arm_mat_init_f32(&Matrix_X_prev,4,1,(float32_t*)data_matrix_X_prev);
  arm_mat_init_f32(&Matrix_A,4,4,(float32_t*)data_matrix_A);
  arm_mat_init_f32(&Matrix_Q,4,4,(float32_t*)data_matrix_Q);
  arm_mat_init_f32(&Matrix_P,4,4,(float32_t*)data_matrix_P);
  arm_mat_init_f32(&Matrix_P_prev,4,4,(float32_t*)data_matrix_P_prev);
  arm_mat_init_f32(&Matrix_ra,3,3,(float32_t*)data_matrix_ra);
  arm_mat_init_f32(&Matrix_rm,3,3,(float32_t*)data_matrix_rm);
  arm_mat_init_f32(&Matrix_rg,3,3,(float32_t*)data_matrix_rg);
  arm_mat_init_f32(&Matrix_R,8,8,(float32_t*)data_matrix_R);
  arm_mat_init_f32(&Matrix_S,8,8,(float32_t*)data_matrix_S);
  arm_mat_init_f32(&Matrix_H,8,4,(float32_t*)data_matrix_H);
  arm_mat_init_f32(&Matrix_H_T,4,8,(float32_t*)data_H_T);
  arm_mat_init_f32(&Matrix_K,4,8,(float32_t*)data_matrix_K);
  arm_mat_init_f32(&Matrix_I,4,4,(float32_t*)data_matrix_I);
  arm_mat_init_f32(&Matrix_skewX,4,3,(float32_t*)data_matrix_skewX);
  arm_mat_init_f32(&Matrix_skewX_T,3,4,(float32_t*)data_skewX_T); 
  arm_mat_init_f32(&Matrix_temp_12,4,3,(float32_t*)temp_matrix_12);
  arm_mat_init_f32(&Matrix_temp_12_1,4,3,(float32_t*)temp_matrix_12_1);
  arm_mat_init_f32(&Matrix_temp_16,4,4,(float32_t*)temp_matrix_16);
  arm_mat_init_f32(&Matrix_temp_16_1,4,4,(float32_t*)temp_matrix_16_1);
  arm_mat_init_f32(&Matrix_temp_16_T,4,4,(float32_t*)temp_matrix_16_T);
  arm_mat_init_f32(&Matrix_temp_32,8,4,(float32_t*)temp_matrix_32); //8x4 matrix
  arm_mat_init_f32(&Matrix_temp_32_T,4,8,(float32_t*)temp_matrix_32_T); //4x8 matrix
  arm_mat_init_f32(&Matrix_temp_64,8,8,(float32_t*)temp_matrix_64);
}
