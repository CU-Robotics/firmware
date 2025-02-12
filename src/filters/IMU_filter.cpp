#include "IMU_filter.hpp"

void IMU_filter::init_EKF_6axis(IMU_data data){
    // For unit Quaternions, The seperation of garvity is what q_i, q_j, q_k should be
    float invvector = 1.0f/sqrt(data.accel_X * data.accel_X + data.accel_Y * data.accel_Y + data.accel_Z * data.accel_Z);
    x[0] = 0;
    x[1] /= invvector;
    x[2] /= invvector;
    x[3] /= invvector;//This is the initial quaterions

    //Since the weight of Q and R for all element should be the same
    Q = 0.01;
    R = 1;

    P[0] = {1000,0.1,0.1,0.1};
    P[1] = {0.1,1000,0.1,0.1};
    P[2] = {0.1,0.1,1000,0.1};
    P[3] = {0.1,0.1,0.1,1000};    
}

void IMU_filter::step_EKF_6axis(IMU_data data, float dt){

    // For unit Quaternions, The seperation of garvity is what q_i, q_j, q_k should be
    float invvector = 1.0f/sqrt(data.accel_X * data.accel_X + data.accel_Y * data.accel_Y + data.accel_Z * data.accel_Z);
    data.accel_X /= invvector;
    data.accel_Y /= invvector;
    data.accel_Z /= invvector;//This is the unit gravity

    //Helping numbers for F
    float helpgx = (data.gyro_X * dt) * 0.5f;
    float helpgy = (data.gyro_Y * dt) * 0.5f;
    float helpgz = (data.gyro_Z * dt) * 0.5f;
    // Predict for x
    x[0]= x[0]          - x[1]*helpgx   - x[2]*helpgy   - x[3]*helpgz;
    x[1]= x[0]*helpgx   + x[1]          + x[2]*helpgz   - x[3]*helpgy;
    x[2]= x[0]*helpgy   - x[1]*helpgz   + x[2]          + x[3]*helpgx;
    x[3]= x[0]*helpgz   + x[1]*helpgy   - x[2]*helpgx   + x[3];
    // predict for F 
    P[0][0] += P[0][0] + 


    // [        1, -(dt*gx)/2, -(dt*gy)/2, -(dt*gz)/2]
    // [(dt*gx)/2,          1,  (dt*gz)/2, -(dt*gy)/2]
    // [(dt*gy)/2, -(dt*gz)/2,          1,  (dt*gx)/2]
    // [(dt*gz)/2,  (dt*gy)/2, -(dt*gx)/2,          1] Jacobian function F state transition model


    // [-2*q2,  2*q3, -2*q0, 2*q1]
    // [ 2*q1,  2*q0,  2*q3, 2*q2]
    // [ 2*q0, -2*q1, -2*q2, 2*q3] Jacobian function H observation model



}


