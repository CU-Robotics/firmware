#include "IMU_filter.hpp"

void IMU_filter::init_EKF_6axis(IMU_data data){
    // For unit Quaternions, The seperation of garvity is what q_i, q_j, q_k should be
    float invvector = 1.0f/sqrt(data.accel_X * data.accel_X + data.accel_Y * data.accel_Y + data.accel_Z * data.accel_Z);
    x[0] = 0;
    x[1] = data.accel_X * invvector;
    x[2] = data.accel_Y * invvector;
    x[3] = data.accel_Z * invvector;//This is the initial quaterions

    //Since the weight of Q and R for all element should be the same
    Q = 0.01;
    R = 1000000;
    chi_square = 0.01;
    P[0] = {10000,0,0,0};
    P[1] = {0,10000,0,0};
    P[2] = {0,0,10000,0};
    P[3] = {0,0,0,10000};    
}

void IMU_filter::step_EKF_6axis(IMU_data data, float dt){
    // For unit Quaternions, The seperation of garvity is what q_i, q_j, q_k should be
    
    float invvector = 1.0f/sqrt(data.accel_X * data.accel_X + data.accel_Y * data.accel_Y + data.accel_Z * data.accel_Z);
    data.accel_X = data.accel_X * invvector;
    data.accel_Y = data.accel_Y * invvector;
    data.accel_Z = data.accel_Z * invvector;//This is the unit gravity

    //Helping numbers for F
    float helpgx = (data.gyro_X ) * 0.5f;
    float helpgy = (data.gyro_Y ) * 0.5f;
    float helpgz = (data.gyro_Z ) * 0.5f;
    Serial.printf("helpgx: %f, helpgy: %f, helpgz: %f\n", helpgx, helpgy, helpgz);
    // Predict for x
    Serial.printf("1.x[0]: %f, x[1]: %f, x[2]: %f, x[3]: %f\n", x[0], x[1], x[2], x[3]);
    Serial.printf("1.pitch: %f",atan2f(2.0f*(x[0]*x[1] + x[2]*x[3]), 2.0f*(x[0]*x[0] - x[3]*x[3] - 1.0f)) * RAD_TO_DEG);
    x[0] = x[0]          - x[1]*helpgx   - x[2]*helpgy   - x[3]*helpgz   ;
    x[1] = x[0]*helpgx   + x[1]          + x[2]*helpgz   - x[3]*helpgy   ;
    x[2] = x[0]*helpgy   - x[1]*helpgz   + x[2]          + x[3]*helpgx   ;
    x[3] = x[0]*helpgz   + x[1]*helpgy   - x[2]*helpgx   + x[3]          ;
    Serial.printf("2.x[0]: %f, x[1]: %f, x[2]: %f, x[3]: %f\n", x[0], x[1], x[2], x[3]);
    Serial.printf("2.pitch: %f",atan2f(2.0f*(x[0]*x[1] + x[2]*x[3]), 2.0f*(x[0]*x[0] - x[3]*x[3] - 1.0f)) * RAD_TO_DEG);
    // predict for P
    float F[4][4] = {
        {1, -helpgx, -helpgy, -helpgz},
        {helpgx, 1, helpgz, -helpgy},
        {helpgy, -helpgz, 1, helpgx},
        {helpgz, helpgy, -helpgx, 1}
    };
    // [        1, -(dt*gx)/2, -(dt*gy)/2, -(dt*gz)/2]
    // [(dt*gx)/2,          1,  (dt*gz)/2, -(dt*gy)/2]
    // [(dt*gy)/2, -(dt*gz)/2,          1,  (dt*gx)/2]
    // [(dt*gz)/2,  (dt*gy)/2, -(dt*gx)/2,          1] Jacobian function F state transition model

    float F_transpose[4][4] = {0};
    float FxP[4][4] = {0};
    float PxF_transpose[4][4] = {0};
    // Calculate FxP and PxF_transpose
    // for(int i = 0; i < 4; i++){
    //     for(int j = 0; j < 4; j++){
    //         for(int k = 0; k < 4; k++){
    //             FxP[i][j] += F[i][k] * P[k][j];
    //             PxF_transpose[i][j] += P[i][k] * F[j][k];  
    //         }
    //     }
    // }
    // for(int i = 0; i < 4; i++){
    //     for(int j = 0; j < 4; j++){
    //         P[i][j] += FxP[i][j] + PxF_transpose[i][j];
    //         if (i == j){
    //             P[i][j] += Q * dt;
    //         }
    //     }
    // }
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            for(int k = 0; k < 4; k++){
                FxP[i][j] += F[i][k] * P[k][j]; 
            }
        }
    }
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            for(int k = 0; k < 4; k++){
                P[i][j] += FxP[i][k] * P[j][k]; 
                if (i == j){
                    P[i][j] += Q * dt;
                }
            }
        }
    }


    
    // Updata
    float H[3][4] = {
        {-2*x[2], 2*x[3], -2*x[0], 2*x[1]},
        {2*x[1], 2*x[0], 2*x[3], 2*x[2]},
        {2*x[0], -2*x[1], -2*x[2], 2*x[3]}
    };
    // [-2*q2,  2*q3, -2*q0, 2*q1]
    // [ 2*q1,  2*q0,  2*q3, 2*q2]
    // [ 2*q0, -2*q1, -2*q2, 2*q3] Jacobian function H observation model

    float PH_transpose[4][3] = {0};
    float HPH_transpose[3][3] = {0};
    // Calculate HPH_transpose
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 4; k++){
                PH_transpose[i][j] += P[i][k] * H[j][k];
            }
        }
    }
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 4; k++){
                HPH_transpose[i][j] += H[i][k] * PH_transpose[k][j];
                if (i == j){
                HPH_transpose[i][j] += R;
                }
            }
        }
    }
    float HPH_transpose_inv[3][3] = {0};
    inverse3x3(HPH_transpose, HPH_transpose_inv);
    float K[4][3] = {0};

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                K[i][j] += PH_transpose[i][k] * HPH_transpose_inv[k][j];
            }
        }
    }
    // caulculate the error between the predicted and the measured
    float g[3] = {
        data.accel_X - 2*(x[1]*x[3] - x[0]*x[2]),
        data.accel_Y - 2*(x[2]*x[3] + x[0]*x[1]),
        data.accel_Z - 1 - 2*(x[1]*x[1] + x[2]*x[2]) 
    };
    float temp[3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i] += g[j] * HPH_transpose_inv[j][i];
        }
    }
    float r = temp[0] * g[0] + temp[1] * g[1] + temp[2] * g[2];
    if (r < 0.1){
        // Updata x
        for(int i = 0; i < 3; i++){
            x[i] += K[i][0] * g[0] + K[i][1] * g[1] + K[i][2] * g[2];
        }
        // Update P
        float KH[4][4] = {0};
        float I_KH[4][4] = {0};
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                for(int k = 0; k < 3; k++){
                    KH[i][j] += K[i][k] * H[k][j];
                }
            }
        }
        float temp2[4][4] = {0};
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                for(int k = 0; k < 4; k++){
                    temp2[i][j] += KH[i][j] * P[k][j];
                }
            }
        }
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                P[i][j] -= temp2[i][j];
            }
        }
    }
    float invqua = 1/sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
    x[0] = x[0] * invqua;
    x[1] = x[1] * invqua;
    x[2] = x[2] * invqua;
    x[3] = x[3] * invqua;

    filtered_data = data;
    filtered_data.pitch = atan2f(2.0f*(x[0]*x[1] + x[2]*x[3]), 2.0f*(x[0]*x[0] - x[3]*x[3] - 1.0f)) * RAD_TO_DEG;
    filtered_data.roll = asinf(-2.0f*(x[1]*x[3] - x[0]*x[2]))* RAD_TO_DEG;
    filtered_data.yaw = atan2f(2.0f*(x[0]*x[3] + x[1]*x[2]), 2.0f*(x[0]*x[0] - x[1]*x[1] - 1.0f))* RAD_TO_DEG;
}

IMU_data* IMU_filter::get_filter_data(){
    return &filtered_data;   
}

void IMU_filter::inverse3x3(float mat[3][3], float inv[3][3]) {
    float det = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) -
                mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
                mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);

    if (det == 0) {
        // Matrix is not invertible
        return;
    }

    float invDet = 1.0f / det;

    inv[0][0] = (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) * invDet;
    inv[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) * invDet;
    inv[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * invDet;
    inv[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) * invDet;
    inv[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * invDet;
    inv[1][2] = (mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2]) * invDet;
    inv[2][0] = (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]) * invDet;
    inv[2][1] = (mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1]) * invDet;
    inv[2][2] = (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]) * invDet;
}





