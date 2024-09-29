#include "BalancingIMU_Filter.hpp"

void IMU_filter::read(){
    imu.accel_X = icm.get_accel_X();
    imu.accel_Y = icm.get_accel_Y();
    imu.accel_Z = icm.get_accel_Z();

    imu.gyro_X = icm.get_gyro_X();
    imu.gyro_Y = icm.get_gyro_Y();
    imu.gyro_Z = icm.get_gyro_Z();

    imu.temperature = icm.get_temperature();
//Something somthing need to be test
}