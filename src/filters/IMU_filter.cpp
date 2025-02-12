#include "IMU_filter.hpp"

void IMU_filter::init_EKF_6axis(IMU_data data){
    // For unit Quaternions, The seperation of garvity is what q_i, q_j, q_k should be
    float invvector = 1.0f/sqrt(data.accel_X * data.accel_X + data.accel_Y * data.accel_Y + data.accel_Z * data.accel_Z);
    x[1] /= invvector;
    x[2] /= invvector;
    x[3] /= invvector;//This is the initial quaterions



}

void IMU_filter::step_EKF_6axis(IMU_data data){

    // For unit Quaternions, The seperation of garvity is what q_i, q_j, q_k should be
    float invvector = 1.0f/sqrt(data.accel_X * data.accel_X + data.accel_Y * data.accel_Y + data.accel_Z * data.accel_Z);
    data.accel_X /= invvector;
    data.accel_Y /= invvector;
    data.accel_Z /= invvector;//This is the unit gravity

    // [        1, -(dt*gx)/2, -(dt*gy)/2, -(dt*gz)/2]
    // [(dt*gx)/2,          1,  (dt*gz)/2, -(dt*gy)/2]
    // [(dt*gy)/2, -(dt*gz)/2,          1,  (dt*gx)/2]
    // [(dt*gz)/2,  (dt*gy)/2, -(dt*gx)/2,          1]



    // [-2*q2,  2*q3, -2*q0, 2*q1]
    // [ 2*q1,  2*q0,  2*q3, 2*q2]
    // [ 2*q0, -2*q1, -2*q2, 2*q3] Jacobian function F state transition model 



}


