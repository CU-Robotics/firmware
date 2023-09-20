#ifndef ICMSENSOR_H
#define ICMSENSOR_H

class IMUSensor {
public:
    IMUSensor();
    void read();
    void read_accel();
    void read_gyro();

    float get_temperature(){ return temperature; };
    
    float get_accel_X() { return accel_X; };
    float get_accel_Y() { return accel_Y; };
    float get_accel_Z() { return accel_Z; };

    float get_gyro_X() { return gyro_X; };
    float get_gyro_Y() { return gyro_Y; };
    float get_gyro_Z() { return gyro_Z; };

private:
    float accel_X = 0;
    float accel_Y = 0;
    float accel_Z = 0;

    float gyro_X = 0;
    float gyro_Y = 0;
    float gyro_Z = 0;

    float temperature = 0;
};

#endif