#include "estimator_manager.hpp"


EstimatorManager::EstimatorManager(CANData *data){

    pinMode(YAW_BUFF_CS, OUTPUT);
    pinMode(PITCH_BUFF_CS, OUTPUT);
    pinMode(ICM_CS, OUTPUT);

    digitalWrite(YAW_BUFF_CS, HIGH);
    digitalWrite(PITCH_BUFF_CS, HIGH);
    digitalWrite(ICM_CS, HIGH);

    Serial.println("Starting SPI");
    SPI.begin();
    Serial.println("SPI Started");

    buff_sensors[0].init(YAW_BUFF_CS);
    buff_sensors[1].init(PITCH_BUFF_CS);

    icm_sensors[0].init(icm_sensors[0].CommunicationProtocol::SPI);
    can_data = data;
}

void EstimatorManager::init_estimator(int state_id,int num_states) {
    switch (state_id)
    {
    case 1:
        float values_chassis[8];
        estimators[0] = new ChassisEstimator(values_chassis, &buff_sensors[0], &buff_sensors[1], &icm_sensors[0], can_data, num_states); 
        break;
    case 2: // Gimbal Estimator
        float values_gimbal[10];
        values_gimbal[0] = 0; // yaw encoder offset
        values_gimbal[1] = 1; // pitch encoder offset
        values_gimbal[2] = 0; // default yaw starting angle (starting point for imu integration)
        values_gimbal[3] = 1.91986; // default pitch starting angle (starting point for imu integration)
        values_gimbal[4] = 0; // default roll starting angle (starting point for imu integration)
        values_gimbal[5] = 0; // default chassis pitch angle 
        // Stable gravity vector {x,y,z}
        values_gimbal[6] = 0.077535; // x
        values_gimbal[7] = 2.396863; // y
        values_gimbal[8] = -6.940948; // z
        values_gimbal[9] = 1.91986; // Pitch angle at given gravity vector
        
        estimators[1] = new GimbalEstimator(values_gimbal, &buff_sensors[0], &buff_sensors[1], &icm_sensors[0], can_data, num_states); 
        break;
    case 3:
        estimators[2] = new FlyWheelEstimator(can_data, num_states); 
        break;
    case 4:
        estimators[3] = new FeederEstimator(can_data, num_states); 
        break;
    case 5:
        estimators[5] = new LocalEstimator(can_data, num_states); 
        break;
    default:
        break;
    }
}

void EstimatorManager::step(float global_outputs[STATE_LEN][3], float local_outputs[STATE_LEN][3]) {
    for (int i = 0;i<STATE_LEN;i++) {
        for (int j = 0;j<3;j++) {
            global_outputs[i][j] = 0;
            local_outputs[i][j] = 0;
        }
    }

    for(int i = 0; i < NUM_ESTIMATORS; i++){
        int num_states = estimators[i]->get_num_states();
        float global_states[STATE_LEN][3];
        float local_states[STATE_LEN][3];
        if (!estimators[i]->local_estimator){
            estimators[i]->step_states(states);
            for(int j = 0; j < num_states; j++){
                global_outputs[applied_states[i][j]][0] = global_outputs[applied_states[i][j]][0] + global_states[j][0];
                global_outputs[applied_states[i][j]][1] = global_outputs[applied_states[i][j]][1] + global_states[j][1];
                global_outputs[applied_states[i][j]][2] = global_outputs[applied_states[i][j]][2] + global_states[j][2];
            }
        }else {
            estimators[i]->step_states(states);
            for(int j = 0; j < num_states; j++) {
                local_outputs[applied_states[i][j]][0] = local_outputs[applied_states[i][j]][0] + local_states[j][0];
                local_outputs[applied_states[i][j]][1] = local_outputs[applied_states[i][j]][1] + local_states[j][1];
                local_outputs[applied_states[i][j]][2] = local_outputs[applied_states[i][j]][2] + local_states[j][2];
            }
        }
    }
}

void EstimatorManager::assign_states(int as[NUM_ESTIMATORS][STATE_LEN]){
    for(int i = 0; i < NUM_ESTIMATORS; i++){
        for(int j = 0; j < STATE_LEN; j++){
            applied_states[i][j] = as[i][j];
        } 
    }
}

void EstimatorManager::read_sensors()
{
    buff_sensors[0].read();
    buff_sensors[1].read();
    icm_sensors[0].read();
}

void EstimatorManager::calibrate_imus(){ 
    Serial.println("Calibrating IMU's...");  
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    float sum_accel_x = 0;
    float sum_accel_y = 0;
    float sum_accel_z = 0;

    for(int i = 0; i < NUM_IMU_CALIBRATION; i++){  
        icm_sensors[0].read(); 
        sum_x += icm_sensors[0].get_gyro_X();
        sum_y += icm_sensors[0].get_gyro_Y();
        sum_z += icm_sensors[0].get_gyro_Z();

        sum_accel_x += icm_sensors[0].get_accel_X();
        sum_accel_y += icm_sensors[0].get_accel_Y();
        sum_accel_z += icm_sensors[0].get_accel_Z();
    }

    Serial.printf("Calibrated offsets: %f, %f, %f", sum_accel_x/NUM_IMU_CALIBRATION, sum_accel_y/NUM_IMU_CALIBRATION, sum_accel_z/NUM_IMU_CALIBRATION);
    Serial.println();
    icm_sensors[0].set_offsets(sum_x/NUM_IMU_CALIBRATION, sum_y/NUM_IMU_CALIBRATION, sum_z/NUM_IMU_CALIBRATION);

    // sum_x = 0;
    // sum_y = 0;
    // sum_z = 0;

    // for(int i = 0; i < NUM_IMU_CALIBRATION; i++){  
    //     icm_sensors[0].read(); 
    //     sum_x += icm_sensors[0].get_gyro_X();
    //     sum_y += icm_sensors[0].get_gyro_Y();
    //     sum_z += icm_sensors[0].get_gyro_Z();
    // }
    // Serial.printf("Calibrated offsets 2: %f, %f, %f", sum_x/NUM_IMU_CALIBRATION, sum_y/NUM_IMU_CALIBRATION, sum_z/NUM_IMU_CALIBRATION);
    // Serial.println();
}

EstimatorManager::~EstimatorManager(){
    Serial.println("Ending SPI");
    SPI.end();
    Serial.println("SPI Ended");

    for(int i = 0; i < STATE_LEN;i++) {
        if(estimators[i] == nullptr) continue;
        delete estimators[i];
    }
}