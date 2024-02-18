
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

        // CPP program to calculate solutions of linear
        // equations using cramer's rule
        
        // This functions finds the determinant of Matrix
        double determinantOfMatrix(double mat[3][3])
        {
            double ans;
            ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
                - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
                + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
            return ans;
        }
        
        // This function finds the solution of system of
        // linear equations using cramer's rule
        // input is 3x3 coeff matrix with 3x1 solution matrix added to the end
        void findSolution(double coeff[3][4], float output[3])
        {
            // Matrix d using coeff as given in cramer's rule
            double d[3][3] = {{ coeff[0][0], coeff[0][1], coeff[0][2] },
                { coeff[1][0], coeff[1][1], coeff[1][2] },
                { coeff[2][0], coeff[2][1], coeff[2][2] }};
            // Matrix d1 using coeff as given in cramer's rule
            double d1[3][3] = {
                { coeff[0][3], coeff[0][1], coeff[0][2] },
                { coeff[1][3], coeff[1][1], coeff[1][2] },
                { coeff[2][3], coeff[2][1], coeff[2][2] },
            };
            // Matrix d2 using coeff as given in cramer's rule
            double d2[3][3] = {
                { coeff[0][0], coeff[0][3], coeff[0][2] },
                { coeff[1][0], coeff[1][3], coeff[1][2] },
                { coeff[2][0], coeff[2][3], coeff[2][2] },
            };
            // Matrix d3 using coeff as given in cramer's rule
            double d3[3][3] = {
                { coeff[0][0], coeff[0][1], coeff[0][3] },
                { coeff[1][0], coeff[1][1], coeff[1][3] },
                { coeff[2][0], coeff[2][1], coeff[2][3] },
            };
        
            // Calculating Determinant of Matrices d, d1, d2, d3
            double D = determinantOfMatrix(d);
            double D1 = determinantOfMatrix(d1);
            double D2 = determinantOfMatrix(d2);
            double D3 = determinantOfMatrix(d3);
            
            // Case 1
            if (D != 0) {
                // Coeff have a unique solution
                output[0] = D1 / D;
                output[1] = D2 / D;
                output[2] = D3 / D;
            }
            // Case 2
            else {
                if (D1 == 0 && D2 == 0 && D3 == 0)
                    Serial.println("matrix solve bad1");
                else if (D1 != 0 || D2 != 0 || D3 != 0)
                    Serial.println("matrix solve bad2");
            }     
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
        float yaw_axis_global[3];
        float pitch_axis_global[3];
        float roll_axis_global[3];
        float current_yaw_velocity = 0;
        float previous_yaw_velocity = 0; 
        float current_pitch_velocity = 0;
        float previous_pitch_velocity = 0; 
        float current_roll_velocity = 0;
        float previous_roll_velocity = 0;
        float global_yaw_velocity = 0;
        float global_roll_velocity = 0;
        float global_pitch_velocity = 0;
        float global_pitch_angle = 1.92;
        float global_yaw_angle = 0;
        float global_roll_angle = 0;
        float dt = 0;
        BuffEncoder *buff_enc_yaw;
        BuffEncoder *buff_enc_pitch;
        CANData *can_data;
        ICM20649 *icm_imu;
    public:
        GimbalEstimator(float sensor_values[10], BuffEncoder *b1, BuffEncoder *b2, ICM20649* imu, CANData* data, int n){
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
            else if (gravity_accel_vector[0] < 0){
                yaw_axis_spherical[1] = PI+atan(gravity_accel_vector[1]/gravity_accel_vector[0]); // theta
            }else{
                yaw_axis_spherical[1] = atan(gravity_accel_vector[1]/gravity_accel_vector[0]); // theta
            }
            yaw_axis_spherical[2] = acos(gravity_accel_vector[2]/__magnitude(gravity_accel_vector,3))+pitch_diff; // phi

            // roll_axis_spherical[1] = yaw_axis_spherical[1]; // theta 
            // roll_axis_spherical[2] = yaw_axis_spherical[2]-(PI*0.5); // phi

            // pitch_axis_spherical[1] = yaw_axis_spherical[1]-(PI*0.5);
            // pitch_axis_spherical[2] = (PI*0.5);

            // convert spherical to cartesian, These unit vectors are axis in the gimbal refrence frame
            yaw_axis_unitvector[0] = yaw_axis_spherical[0]*cos(yaw_axis_spherical[1])*sin(yaw_axis_spherical[2]); 
            yaw_axis_unitvector[1] = yaw_axis_spherical[0]*sin(yaw_axis_spherical[1])*sin(yaw_axis_spherical[2]);
            yaw_axis_unitvector[2] = yaw_axis_spherical[0]*cos(yaw_axis_spherical[2]);

            // roll_axis_unitvector[0] = roll_axis_spherical[0]*cos(roll_axis_spherical[1])*sin(roll_axis_spherical[2]); 
            // roll_axis_unitvector[1] = roll_axis_spherical[0]*sin(roll_axis_spherical[1])*sin(roll_axis_spherical[2]);
            // roll_axis_unitvector[2] = roll_axis_spherical[0]*cos(roll_axis_spherical[2]);

            // pitch_axis_unitvector[0] = pitch_axis_spherical[0]*cos(pitch_axis_spherical[1])*sin(pitch_axis_spherical[2]); 
            // pitch_axis_unitvector[1] = pitch_axis_spherical[0]*sin(pitch_axis_spherical[1])*sin(pitch_axis_spherical[2]);
            // pitch_axis_unitvector[2] = pitch_axis_spherical[0]*cos(pitch_axis_spherical[2]);
            float mag = sqrt((-2.889366*-2.889366)+(-0.026573*-0.026573)+(-0.023502*-0.023502));
            pitch_axis_unitvector[0] = -2.889366/mag;
            pitch_axis_unitvector[1] = -0.026573/mag;
            pitch_axis_unitvector[2] = -0.023502/mag;

            __crossProduct(pitch_axis_unitvector,yaw_axis_unitvector,roll_axis_unitvector);

            float magicNum = 0.005; // left yaw increases with 0.8
            __rotateVector3D(roll_axis_unitvector,yaw_axis_unitvector,magicNum,yaw_axis_unitvector);
            __rotateVector3D(roll_axis_unitvector,pitch_axis_unitvector,magicNum,pitch_axis_unitvector);

            // __rotateVector3D(yaw_axis_unitvector,roll_axis_unitvector,(PI*0.5),pitch_axis_unitvector);

            // offset the axis' based on the pitch yaw and roll data, These vectors give global pitch yaw and roll
            __rotateVector3D(roll_axis_unitvector,yaw_axis_unitvector,global_roll_angle,yaw_axis_global);
            __rotateVector3D(roll_axis_unitvector,pitch_axis_unitvector,global_roll_angle,pitch_axis_global);

            __rotateVector3D(pitch_axis_unitvector,yaw_axis_unitvector,(global_pitch_angle-angle),yaw_axis_global);
            __rotateVector3D(pitch_axis_unitvector,roll_axis_unitvector,(global_pitch_angle-angle),roll_axis_global);
            
            // gets the velocity data from the imu and uses the gravity vector to calculate the yaw velocity
            float raw_omega_vector[3] = { icm_imu->get_gyro_X(),icm_imu->get_gyro_Y(), icm_imu->get_gyro_Z()};
            // *Note: X is pitch Y is Roll Z is Yaw, when level
            // positive pitch angle is up, positive roll angle is right(robot pov), positive yaw is left(robot pov)
                
                // Serial.print(pitch_axis_unitvector[0]);
                // Serial.print(", ");
                // Serial.print(pitch_axis_unitvector[1]);
                // Serial.print(", ");
                // Serial.println(pitch_axis_unitvector[2]);

                // Serial.printf("info: %f, %f, %f", pitch_axis_unitvector[0], pitch_axis_unitvector[1], pitch_axis_unitvector[2]);
                // Serial.println();
                // Serial.print(roll_angle);
                // Serial.print(", ");
                // Serial.print(pitch_angle);
                // Serial.print(", ");
                // Serial.println(yaw_angle);

            // Serial.printf("angles: %f, %f, %f", roll_angle, pitch_angle, yaw_angle);
            // Serial.println();

            float temp1[3];
            float temp2[3];
            float temp3[3];
            __crossProduct(yaw_axis_unitvector,pitch_axis_unitvector,temp1);
            __crossProduct(yaw_axis_unitvector,roll_axis_unitvector,temp2);
            __crossProduct(roll_axis_unitvector,pitch_axis_unitvector,temp3);
                // Serial.printf("cp: %f, %f, %f", __magnitude(temp1,3), __magnitude(temp2,3), __magnitude(temp3,3));
                // Serial.println();

            // update previous to the current value before current is updated
            previous_pitch_velocity = current_pitch_velocity;
            previous_yaw_velocity = current_yaw_velocity;
            previous_roll_velocity = current_roll_velocity;

            // calculate the pitch yaw and roll velocities (Gimbal Relative)
            current_pitch_velocity = __vectorProduct(pitch_axis_unitvector,raw_omega_vector, 3);
            current_yaw_velocity = __vectorProduct(yaw_axis_unitvector, raw_omega_vector, 3);
            current_roll_velocity = __vectorProduct(roll_axis_unitvector, raw_omega_vector, 3);

            //calculate the pitch yaw and roll velocities (Global Reference)
            global_pitch_velocity = __vectorProduct(pitch_axis_global,raw_omega_vector, 3);
            global_yaw_velocity = __vectorProduct(yaw_axis_global, raw_omega_vector, 3);
            global_roll_velocity = __vectorProduct(roll_axis_global, raw_omega_vector, 3);
        // position integration
            dt = time.delta();
            if (dt > .002) dt = 0; // first dt loop generates huge time so check for that 
            yaw_angle += -current_yaw_velocity*(dt/7.85);
            pitch_angle += current_pitch_velocity*(dt/7.85);
            roll_angle += -current_roll_velocity*(dt/7.85);

            global_yaw_angle += -global_yaw_velocity*(dt/7.85);
            global_pitch_angle += -global_pitch_velocity*(dt/7.85);
            global_roll_angle += -global_roll_velocity*(dt/7.85);

            chassis_angle = yaw_angle - buff_enc_yaw->get_angle();

            output[0][0] = chassis_angle;
            output[0][1] = 0;
            output[0][2] = 0;
            output[1][0] = yaw_angle;
            output[1][1] = current_yaw_velocity;
            output[1][2] = 0;
            output[2][0] = pitch_angle;
            output[2][1] = current_pitch_velocity;
            output[2][2] = 0;
        }
};

struct ChassisEstimator : public Estimator {
    private:
    BuffEncoder *buff_enc_yaw;
    BuffEncoder *buff_enc_pitch;
    CANData *can_data;
    ICM20649 *icm_imu;
    public:
    ChassisEstimator(float sensor_values[9], BuffEncoder *b1, BuffEncoder *b2, ICM20649* imu, CANData* data, int n) {
    num_states = n; // number of estimated states
    buff_enc_yaw = b1; // sensor object definitions
    buff_enc_pitch = b2;
    can_data = data;
    icm_imu = imu;
    }

    void step_states(float output[STATE_LEN][3]) override{
        float front_right = can_data->get_motor_attribute(CAN_1,1,MotorAttribute::SPEED);
        float back_right = can_data->get_motor_attribute(CAN_1,2,MotorAttribute::SPEED);
        float back_left = can_data->get_motor_attribute(CAN_1,3,MotorAttribute::SPEED);
        float front_left = can_data->get_motor_attribute(CAN_1,4,MotorAttribute::SPEED);

        output[0][0] = 0; // x pos
        output[0][1] = 0;
        output[0][2] = 0;
        output[1][0] = 0; // y pos
        output[1][1] = 0;
        output[1][2] = 0;
        output[2][0] = 0; // chassis angle
        output[2][1] = ((front_right/60)*(PI*0.5*50))/200;
        output[2][2] = 0;
    }
};
#endif

 