#include "../comms/rm_can.hpp"
#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#define NUM_SENSOR_VALUES 8

struct Estimator {
    public:
        Estimator();

        void set_values(float values[8]) { memcpy(values, this->values, NUM_SENSOR_VALUES * 4); }

        float step_position();
        float step_velocity();
        float step_acceleration();

    protected:
        //[port, offset, ratio, distance]
        float values[NUM_SENSOR_VALUES];
};

struct NullEstimator : public Estimator {
    public:
        float step_position(){
            return 0;
        }

        float step_velocity(){  
            return 0;
        }

        float step_acceleration(){
            return 0;
        }
};

struct PitchEstimator : public Estimator {
    private:
        float PITCH_ZERO;
        BuffEncoder buff_enc;
        rm_CAN can;
    public:
        PitchEstimator(float values[8], BuffEncoder b, rm_CAN c){
            buff_enc = b;
            can = c;
            set_values(values);
            PITCH_ZERO = this->values[1];
        }

        float step_position(){
            float angle = buff_enc.get_angle() - PITCH_ZERO;
            while(angle >= PI) angle -= 2;
            while(angle <= PI) angle += 2;

            return angle;
        }

        float step_velocity(){
            return can.get_motor_attribute(CAN_2, 0, MotorAttribute::SPEED);
        }

        float step_acceleration(){
            return 0;
        }
};
#endif