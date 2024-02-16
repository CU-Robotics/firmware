
#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include "../comms/rm_can.hpp"
#include "state.hpp"

#define NUM_SENSOR_VALUES 8

struct Estimator {   
    public:
        Estimator() {};

        //Virtual so they don't ever get called over real estimators
        virtual void step_states(float outputs[STATE_LEN][3]);

        int get_num_states(){return num_states;}

    protected:
        int num_states;
        
        float __magnitude(float* a, int n) {
            float square_sum = 0;
            for (int i = 0; i < n;i++) {
                square_sum += pow(a[i], 2);
            }
            square_sum = sqrt(square_sum);
            return square_sum;
        }

        float __vectorProduct(float* a, float* b, int n) {
            float product = 0;
            for (int i = 0; i < n; i++) {
                product += a[i] * b[i];
            }
            return product;
        }
        
        void __crossProduct(float v_A[], float v_B[], float output[]) {
            output[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
            output[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
            output[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
        }

        void __rotateVector3D(float unit_vector[], float input_vector[], float theta, float output[]) {
            float unit_cross_input[3];
            __crossProduct(unit_vector,input_vector,unit_cross_input);
            output[0] = (input_vector[0]*cos(theta)) + (unit_cross_input[0]*sin(theta)) + (unit_vector[0]*__vectorProduct(unit_vector,input_vector,3)*(1-cos(theta)));
            output[1] = (input_vector[1]*cos(theta)) + (unit_cross_input[1]*sin(theta)) + (unit_vector[1]*__vectorProduct(unit_vector,input_vector,3)*(1-cos(theta)));
            output[2] = (input_vector[2]*cos(theta)) + (unit_cross_input[2]*sin(theta)) + (unit_vector[2]*__vectorProduct(unit_vector,input_vector,3)*(1-cos(theta)));
        }

        Timer time;      
};

struct GimbalEstimator : public Estimator {
    private:
        float YAW_ENCODER_OFFSET; // input variables
        float PITCH_ENCODER_OFFSET;
        float pitch_angle;
        float yaw_angle;
        float roll_angle;
        float chassis_angle;
        float chasis_pitch_angle;
        float gravity_accel_vector[3];
        float gravity_pitch_angle;
        float yaw_axis_spherical[3];
        float pitch_axis_spherical[3];
        float roll_axis_spherical[3];
        float yaw_axis_unitvector[3];
        float pitch_axis_unitvector[3];
        float roll_axis_unitvector[3];
        float yaw_axis_offset[3];
        float pitch_axis_offset[3];
        float roll_axis_offset[3];
        float current_yaw_velocity = 0;
        float previous_yaw_velocity = 0; 
        float current_pitch_velocity = 0;
        float previous_pitch_velocity = 0; 
        float current_roll_velocity = 0;
        float previous_roll_velocity = 0;
        float dt = 0;
        BuffEncoder *buff_enc_yaw;
        BuffEncoder *buff_enc_pitch;
        CANData *can_data;
        ICM20649 *icm_imu;
    public:
        GimbalEstimator(float sensor_values[9], BuffEncoder *b1, BuffEncoder *b2, ICM20649* imu, CANData* data, int n){
            buff_enc_yaw = b1; // sensor object definitions
            buff_enc_pitch = b2;
            can_data = data;
            icm_imu = imu;
            num_states = n; // number of estimated states
            PITCH_ENCODER_OFFSET = sensor_values[1];
            YAW_ENCODER_OFFSET = sensor_values[0];
            pitch_angle = sensor_values[3];
            yaw_angle = sensor_values[2];
            roll_angle = sensor_values[4];
            chasis_pitch_angle = sensor_values[5];
            chassis_angle = 0;
            gravity_accel_vector[0] = sensor_values[6];
            gravity_accel_vector[1] = sensor_values[7];
            gravity_accel_vector[2] = sensor_values[8];
            gravity_pitch_angle = sensor_values[9];
            // definitions for spherical coordinates of new axis in the imu refrence frame
            yaw_axis_spherical[0] = 1; // rho (1 for a spherical)
            pitch_axis_spherical[0] = 1; // rho (1 for a spherical)
            roll_axis_spherical[0] = 1; // rho (1 for a spherical)
        }
        
        void step_states(float output[STATE_LEN][3]) override{         
            float angle = (-buff_enc_pitch->get_angle()) + PITCH_ENCODER_OFFSET;
            while(angle >= PI) angle -= 2 * PI;
            while(angle <= -PI) angle += 2 * PI;   
        // calculates yaw velocity before integrating to find position
            // calculates the difference in initial and current pitch angle
            float pitch_diff = gravity_pitch_angle - angle;
            
            // gimbal rotation axis in spherical coordinates in imu refrence frame
            if (gravity_accel_vector[0] == 0) yaw_axis_spherical[1] = 1.57;
            else {
                yaw_axis_spherical[1] = atan(gravity_accel_vector[1]/gravity_accel_vector[0]); // theta
            }
            yaw_axis_spherical[2] = acos(gravity_accel_vector[2]/__magnitude(gravity_accel_vector,3))+pitch_diff; // phi

            // pitch_axis_spherical[1] = yaw_axis_spherical[1]-(PI*0.5); // theta
            // pitch_axis_spherical[2] = (PI*0.5); // phi

            roll_axis_spherical[1] = yaw_axis_spherical[1]; // theta 
            roll_axis_spherical[2] = yaw_axis_spherical[2]-(PI*0.5); // phi

            // convert spherical to cartesian for imu calcs
            yaw_axis_unitvector[0] = yaw_axis_spherical[0]*cos(yaw_axis_spherical[1])*sin(yaw_axis_spherical[2]); 
            yaw_axis_unitvector[1] = yaw_axis_spherical[0]*sin(yaw_axis_spherical[1])*sin(yaw_axis_spherical[2]);
            yaw_axis_unitvector[2] = yaw_axis_spherical[0]*cos(yaw_axis_spherical[2]);

            roll_axis_unitvector[0] = roll_axis_spherical[0]*cos(roll_axis_spherical[1])*sin(roll_axis_spherical[2]); 
            roll_axis_unitvector[1] = roll_axis_spherical[0]*sin(roll_axis_spherical[1])*sin(roll_axis_spherical[2]);
            roll_axis_unitvector[2] = roll_axis_spherical[0]*cos(roll_axis_spherical[2]);

            __rotateVector3D(yaw_axis_unitvector,roll_axis_unitvector,(PI*0.5),pitch_axis_unitvector);

            // pitch_axis_unitvector[0] = pitch_axis_spherical[0]*cos(pitch_axis_spherical[1])*sin(pitch_axis_spherical[2]);
            // pitch_axis_unitvector[1] = pitch_axis_spherical[0]*sin(pitch_axis_spherical[1])*sin(pitch_axis_spherical[2]);
            // pitch_axis_unitvector[2] = pitch_axis_spherical[0]*cos(pitch_axis_spherical[2]);

            // offset the axis' based on the pitch yaw and roll data
            __rotateVector3D(roll_axis_unitvector,yaw_axis_unitvector,roll_angle,yaw_axis_offset);
            __rotateVector3D(roll_axis_unitvector,pitch_axis_unitvector,roll_angle,pitch_axis_offset);

            __rotateVector3D(pitch_axis_unitvector,yaw_axis_offset,(pitch_angle-angle),yaw_axis_offset);
            __rotateVector3D(pitch_axis_unitvector,roll_axis_unitvector,(pitch_angle-angle),roll_axis_offset);
            
            // gets the velocity data from the imu and uses the gravity vector to calculate the yaw velocity
            float raw_omega_vector[3] = { icm_imu->get_gyro_X(),icm_imu->get_gyro_Y() ,icm_imu->get_gyro_Z()};
            // *Note: X is pitch Y is Roll Z is Yaw, when level
            // positive pitch angle is up, positive roll angle is right(robot pov), positive yaw is left(robot pov)
                
                // Serial.print(pitch_axis_offset[0]);
                // Serial.print(", ");
                // Serial.print(pitch_axis_offset[1]);
                // Serial.print(", ");
                // Serial.println(pitch_axis_offset[2]);

                Serial.print(roll_angle);
                Serial.print(", ");
                Serial.print(pitch_angle);
                Serial.print(", ");
                Serial.println(yaw_angle);

            // update previous to the current value before current is updated
            previous_pitch_velocity = current_pitch_velocity;
            previous_yaw_velocity = current_yaw_velocity;
            previous_roll_velocity = current_roll_velocity;

            // calculate the pitch yaw and roll velocities
            current_pitch_velocity = __vectorProduct(pitch_axis_unitvector,raw_omega_vector, 3);
            current_yaw_velocity = __vectorProduct(yaw_axis_unitvector, raw_omega_vector, 3);
            current_roll_velocity = __vectorProduct(roll_axis_unitvector, raw_omega_vector, 3);

        // position integration
            dt = time.delta();
            if (dt > .002) dt = 0; // first dt loop generates huge time so check for that 
            yaw_angle += -current_yaw_velocity*(dt/7.85);
            pitch_angle += -current_pitch_velocity*(dt/7.85);
            roll_angle += -current_roll_velocity*(dt/7.85);
            chassis_angle = yaw_angle - buff_enc_yaw->get_angle();

            output[0][0] = chassis_angle;
            output[0][1] = roll_angle;
            output[0][2] = current_roll_velocity;
            output[1][0] = yaw_angle;
            output[1][1] = current_yaw_velocity;
            output[1][2] = 0;
            output[2][0] = pitch_angle;
            output[2][1] = current_pitch_velocity;
            output[2][2] = 0;
        }
};













// struct PitchEstimator : public Estimator {
//     private:
//         float PITCH_ZERO;
//         BuffEncoder *buff_enc;
//         CANData *can_data;
//         ICM20649 *icm_imu;
//     public:
//         PitchEstimator(float sensor_values[8], BuffEncoder *b, ICM20649* imu, CANData* data){
//             buff_enc = b;
//             can_data = data;
//             set_values(sensor_values);
//             PITCH_ZERO = this->sensor_values[1];
//             icm_imu = imu;
//         }
        
//         float step_position() override{
//             float angle = (-buff_enc->get_angle()) + PITCH_ZERO;
//             while(angle >= PI) angle -= 2 * PI;
//             while(angle <= -PI) angle += 2 * PI;
//             return angle;
//         }

//         float step_velocity()override{
//             return can_data->get_motor_attribute(CAN_2, 1, MotorAttribute::SPEED);
//         }

//         float step_acceleration()override{
//             return icm_imu->get_gyro_Y();
//         }
// };

// struct YawEstimator : public Estimator {
//     private:
//         float YAW_ZERO;
//         float PITCH_ZERO;
//         BuffEncoder *buff_enc;
//         CANData *can_data;
//         ICM20649 *icm_imu;
//         float previous_velocity = 0;
//         float current_velocity = 0;
//         // defines the vector for gravity at a given pitch angle
//         float initial_accel_vector[3] = {0,0,1};
//         float initial_pitch_angle = 1.57079;
//     public:
//         YawEstimator(float values[8], BuffEncoder *b, ICM20649* imu, CANData* data){
//             buff_enc = b;
//             can_data = data;
//             set_values(values);
//             YAW_ZERO = this->values[1];
//             PITCH_ZERO = this->values[2];
//             icm_imu = imu;
//         }
        
//         float step_position() override{
//         // calulate pitch angle
//         // *note: this doesnt make sense to do again we should just pass it in
//             float pitch_angle = (-buff_enc->get_angle()) + PITCH_ZERO;
//             while(pitch_angle >= PI) pitch_angle -= 2 * PI;
//             while(pitch_angle <= -PI) pitch_angle += 2 * PI;
        
//         // calculates yaw velocity before integrating to find position
//             // calculates the difference in initial and current pitch angle
//             float pitch_diff = initial_pitch_angle - pitch_angle;
//             float ground_pointing_spherical[3] = { 0 };

//             // converts the initial gravity vector into the current gravity vector in spherical coordinates
//             ground_pointing_spherical[0] = __magnitude(initial_accel_vector, 3);
//             ground_pointing_spherical[1] = atan(initial_accel_vector[0]/initial_accel_vector[1]);
//             ground_pointing_spherical[2] = acos(initial_accel_vector[2]/ground_pointing_spherical[0])+pitch_diff;

//             // gets the velocity data from the imu and uses the gravity vector to calculate the yaw velocity
//             float raw_omega_vector[3] = { icm_imu->get_gyro_X(),icm_imu->get_gyro_Y() ,icm_imu->get_gyro_Z() };
//             // update previous to the current value before current is updated
//             previous_velocity = current_velocity;
//             current_velocity = __vectorProduct(ground_pointing_spherical, raw_omega_vector, 3);

//         // position integration
//             //
//             float yaw_angle = 0;

//             return yaw_angle;
//         }

//         float step_velocity()override{
            
//             return current_velocity;
//         }

//         float step_acceleration()override{
//             float accel = (current_velocity - previous_velocity)/time.delta();
//             return accel;
//         }
// };
#endif