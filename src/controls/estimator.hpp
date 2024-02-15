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

        float __magnitude(float* a, int n) {
            float square_sum = 0;
            for (int i = 0; i < n;i++) {
                square_sum += pow(a[i], 2);
            }
            return square_sum;
        }

        float __vectorProduct(float* a, float* b, int n) {
            float product = 0;
            for (int i = 0; i < n; i++) {
                product += a[i] * b[i];
            }
            return product;
        }
        
        Timer time;
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

struct YawEstimator : public Estimator {
    private:
        float YAW_ZERO;
        float PITCH_ZERO;
        BuffEncoder *buff_enc;
        CANData *can_data;
        ICM20649 *icm_imu;
        float previous_velocity = 0;
        float current_velocity = 0;
        // defines the vector for gravity at a given pitch angle
        float initial_accel_vector[3] = {0,0,1};
        float initial_pitch_angle = 1.57079;
    public:
        YawEstimator(float values[8], BuffEncoder *b, ICM20649* imu, CANData* data){
            buff_enc = b;
            can_data = data;
            set_values(values);
            YAW_ZERO = this->values[1];
            PITCH_ZERO = this->values[2];
            icm_imu = imu;
        }
        
        float step_position() override{
        // calulate pitch angle
        // *note: this doesnt make sense to do again we should just pass it in
            float pitch_angle = (-buff_enc->get_angle()) + PITCH_ZERO;
            while(pitch_angle >= PI) pitch_angle -= 2 * PI;
            while(pitch_angle <= -PI) pitch_angle += 2 * PI;
        
        // calculates yaw velocity before integrating to find position
            // calculates the difference in initial and current pitch angle
            float pitch_diff = initial_pitch_angle - pitch_angle;
            float ground_pointing_unitvector[3] = { 0 };

            // converts the initial gravity vector into the current gravity vector in spherical coordinates
            ground_pointing_unitvector[0] = __magnitude(initial_accel_vector, 3);
            ground_pointing_unitvector[1] = atan(initial_accel_vector[0]/initial_accel_vector[1]);
            ground_pointing_unitvector[2] = acos(initial_accel_vector[2]/ground_pointing_unitvector[0])+pitch_diff;

            // gets the velocity data from the imu and uses the gravity vector to calculate the yaw velocity
            float raw_omega_vector[3] = { icm_imu->get_gyro_X(),icm_imu->get_gyro_Y() ,icm_imu->get_gyro_Z() };
            // update previous to the current value before current is updated
            previous_velocity = current_velocity;
            current_velocity = __vectorProduct(ground_pointing_unitvector, raw_omega_vector, 3);

        // position integration
            //
            float yaw_angle = 0;

            return yaw_angle;
        }

        float step_velocity()override{
            
            return current_velocity;
        }

        float step_acceleration()override{
            float accel = (current_velocity - previous_velocity)/time.delta();
            return accel;
        }
};
#endif