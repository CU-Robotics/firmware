#include "IMU_Filter.hpp"

void IMU_filter::init(){
    pinMode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH);
    SPI.begin();
    _icm.init(_icm.SPI);
    _icm.set_gyro_range(4000);
    calibrate_imu();
    _alpha = 0.9995;
    Q = 0.0025;
    R = 0.3;
}

void IMU_filter::read() {
    _icm.read();
    float dt = timer.delta();
    _imu.accel_X = _icm.get_accel_X() * _imu.accel_scale;
    _imu.accel_Y = _icm.get_accel_Y() * _imu.accel_scale; 
    _imu.accel_Z = _icm.get_accel_Z() * _imu.accel_scale;
    _imu.alpha_X = _icm.get_gyro_X();
    _imu.alpha_Y = _icm.get_gyro_Y();
    _imu.alpha_Z = _icm.get_gyro_Z();
    // Calculate roll and pitch angle by gravity
    _imu.accel_pitch = -atanf(_imu.accel_X / sqrt(_imu.accel_Y * _imu.accel_Y + _imu.accel_Z * _imu.accel_Z));
    _imu.accel_roll = atanf(_imu.accel_Y / sqrt(_imu.accel_X * _imu.accel_X + _imu.accel_Z * _imu.accel_Z));

    // Calculate the roll and pitch rotation speed by rotation matrix
    float realalpha_roll = _imu.alpha_X + ((sin(_imu.gyro_pitch) * sin(_imu.gyro_roll)) / cos(_imu.gyro_pitch)) * _imu.alpha_Y + _imu.alpha_Z * ((sin(_imu.gyro_pitch) * cos(_imu.gyro_roll)) / cos(_imu.gyro_pitch));
    float realalpha_pitch = _imu.alpha_Y * cos(_imu.gyro_roll) - _imu.alpha_Z * sin(_imu.gyro_roll);    
    

    // Basic and simple filter to make Accel low pass
    _alpha = 1 - (0.0008 * SENSORS_GRAVITY_EARTH/sqrt((_imu.accel_X*_imu.accel_X) + (_imu.accel_Y*_imu.accel_Y) + (_imu.accel_Z*_imu.accel_Z)));   
    _imu.gyro_roll = _alpha * (_imu.gyro_roll + realalpha_roll * dt) + (1.0 - _alpha) * _imu.accel_roll;
    _imu.gyro_pitch = _alpha * (_imu.gyro_pitch + realalpha_pitch * dt) + (1.0 - _alpha) * _imu.accel_pitch;
    // // Calculate Angle by Gyro
    // _imu.gyro_roll = _imu.k_roll + (dt * realalpha_roll);
    // _imu.gyro_pitch = _imu.k_pitch + (dt * realalpha_pitch);
// Kalman Filter

    // Calculate P Matrix
    P[0][0] = P[0][0] + Q;
    P[0][1] = P[0][1] + 0;
    P[1][0] = P[1][0] + 0;
    P[1][1] = P[1][1] + Q;
    // Update K Matrix
    K[0][0] = P[0][0]/(P[0][0] + R);
    K[0][1] = 0;
    K[1][0] = 0;
    K[1][1] = P[1][1]/(P[1][1] + R);
    // Calculate prediction
    // Roll Angle
    _imu.k_roll = _imu.gyro_roll + K[0][0] * (_imu.accel_roll - _imu.gyro_roll);
    // Pitch Angle
    _imu.k_pitch = _imu.gyro_pitch + K[1][1] * (_imu.accel_pitch - _imu.gyro_pitch);
    // Update P Matrix
    P[0][0] = (1 - K[0][0]) * P[0][0];
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = (1 - K[1][1]) * P[1][1];


    
    
    
    _imu.temperature = _icm.get_temperature();
}

void IMU_filter::print() {
    Serial.print("\t\tTemperature ");
    Serial.print(_imu.temperature);
    Serial.println(" deg C");

    Serial.print("\t\tAccel X: ");
    Serial.print(_imu.accel_X);
    Serial.print(" \tY: ");
    Serial.print(_imu.accel_Y);
    Serial.print(" \tZ: ");
    Serial.print(_imu.accel_Z);
    Serial.println(" m/s^2 ");

    Serial.print("\t\tGyro X: ");
    Serial.print(_imu.alpha_X);
    Serial.print(" \tY: ");
    Serial.print(_imu.alpha_Y);
    Serial.print(" \tZ: ");
    Serial.print(_imu.alpha_Z);
    Serial.println(" radians/s ");

    Serial.println(_imu.gyro_roll * RAD_TO_DEG);
    Serial.println("_imu.gyro_roll");
    Serial.println(_imu.gyro_pitch * RAD_TO_DEG);
    Serial.println("_imu.gyro_pitch");

    Serial.println(_imu.k_roll * RAD_TO_DEG);
    Serial.println("_imu.k_roll");
    Serial.println(_imu.k_pitch * RAD_TO_DEG);
    Serial.println("_imu.k_pitch");

    Serial.println(_imu.accel_roll * RAD_TO_DEG);
    Serial.println("_imu.accel_roll");
    Serial.println(_imu.accel_pitch * RAD_TO_DEG);
    Serial.println("_imu.accel_pitch");
}
void IMU_filter::serial_data_for_plot(){
    String dataString = String(String(_imu.k_roll) + "," + String(_imu.k_pitch));
    Serial.println(dataString);
}
IMUData* IMU_filter::getdata(){
    return &_imu;
}
void IMU_filter::calibrate_imu(){
    Serial.println("Calibrating IMU...");
    float sum_accel_x = 0;
    float sum_accel_y = 0;
    float sum_accel_z = 0;
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    for (int i = 0; i < CALIBRATION_NUM; i++){
        _icm.read();
        sum_accel_x += _icm.get_accel_X();
        sum_accel_y += _icm.get_accel_Y();
        sum_accel_z += _icm.get_accel_Z();
        sum_x += _icm.get_gyro_X();
        sum_y += _icm.get_gyro_Y();
        sum_z += _icm.get_gyro_Z();
    }
    float x = sum_accel_x/CALIBRATION_NUM;
    float y = sum_accel_y/CALIBRATION_NUM;
    float z = sum_accel_z/CALIBRATION_NUM;
    _imu.accel_scale = SENSORS_GRAVITY_EARTH/sqrt((x*x) + (y*y) + (z*z));
    _icm.set_offsets(sum_x / CALIBRATION_NUM, sum_y / CALIBRATION_NUM, sum_z / CALIBRATION_NUM);
    _imu.gyro_pitch=-atanf(x / sqrt(y*y + z*z));
    _imu.gyro_roll=atanf(y / sqrt(z*z));
    _imu.k_pitch = _imu.gyro_pitch;
    _imu.k_roll = _imu.gyro_roll;
}



