#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include "../comms/rm_can.hpp"

#define NUM_SENSOR_VALUES 8

struct Estimator {
    public:
        Estimator() {};

        //Update the values array for specific estimator
        void set_values(float values[8]) { memcpy(this->sensor_values, values, NUM_SENSOR_VALUES * 4); }

        //Virtual so they don't ever get called over real estimators
        virtual float step_position(){return 0;}
        virtual float step_velocity(){return 0;}
        virtual float step_acceleration(){return 0;}

    protected:
        //[port, offset, ratio, distance]
        float sensor_values[NUM_SENSOR_VALUES];
};

struct PitchEstimator : public Estimator {
    private:
        float PITCH_ZERO;
        BuffEncoder *buff_enc;
        CANData *can_data;
        ICM20649 *icm_imu;
    public:
        PitchEstimator(float sensor_values[8], BuffEncoder *b, ICM20649* imu, CANData* data){
            buff_enc = b;
            can_data = data;
            set_values(sensor_values);
            PITCH_ZERO = this->sensor_values[1];
            icm_imu = imu;
        }
        
        float step_position() override{
            float angle = (-buff_enc->get_angle()) + PITCH_ZERO;
            while(angle >= PI) angle -= 2 * PI;
            while(angle <= -PI) angle += 2 * PI;
            return angle;
        }

        float step_velocity()override{
            return can_data->get_motor_attribute(CAN_2, 1, MotorAttribute::SPEED);
        }

        float step_acceleration()override{
            return icm_imu->get_gyro_Y();
        }
};

// struct YawEstimator : public Estimator {
//     private:
//         float PITCH_ZERO;
//         BuffEncoder buff_enc;
//         rm_CAN *can;
//     public:
//         YawEstimator(float values[8], BuffEncoder b, rm_CAN *c){
//             buff_enc = b;
//             can = c;
//             set_values(values);
//             YAW_ZERO = this->values[1];
//         }

//         float step_position(){
//             float angle = buff_enc.get_angle() - YAW_ZERO;
//             while(angle >= PI) angle -= 2;
//             while(angle <= PI) angle += 2;

//             return angle;
//         }

//         float step_velocity(){
//             return can->get_motor_attribute(CAN_1, 0, MotorAttribute::SPEED);
//         }

//         float step_acceleration(){
//             return 0;
//         }
// };
#endif