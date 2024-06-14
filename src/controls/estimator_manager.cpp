#include "estimator_manager.hpp"

EstimatorManager::EstimatorManager(CANData* data, Config c_data) {
    for (int i = 0;i < c_data.num_sensors[0];i++) {
        pinMode(c_data.encoder_pins[i], OUTPUT);
        digitalWrite(c_data.encoder_pins[i], HIGH);
    }

    pinMode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH);

    Serial.println("Starting SPI");
    SPI.begin();
    Serial.println("SPI Started");

    //buff enc loop
    for(int i = 0;i < c_data.num_sensors[0];i++){
        buff_sensors[i].init(c_data.encoder_pins[i]);
    }
    //imu loop
    for(int i = 0;i < c_data.num_sensors[1];i++){
        icm_sensors[i].init(icm_sensors[i].CommunicationProtocol::SPI);
        icm_sensors[i].set_gyro_range(4000);
    }
    //rev enc loop
    for(int i = 0;i < c_data.num_sensors[2];i++){
        rev_sensors[i].init(REV_ENC_PIN1+i,true);
    }
    //time of flight sensor loop
    for(int i = 0;i < c_data.num_sensors[3];i++){
        tof_sensors[i].init();
    }

    can_data = data;
    config_data = c_data;
}

void EstimatorManager::init_estimator(int estimator_id, int num_states) {
    switch (estimator_id) {
    case 1: 
        estimators[num_estimators] = new GimbalEstimator(config_data,&rev_sensors[0],&rev_sensors[1],&rev_sensors[2], &buff_sensors[0], &buff_sensors[1], &icm_sensors[0], can_data, num_states);
        break;

    case 2:
        estimators[num_estimators] = new FlyWheelEstimator(can_data, num_states);
        break;
    case 3:
        estimators[num_estimators] = new FeederEstimator(can_data, num_states);
        break;
    case 4:
        estimators[num_estimators] = new LocalEstimator(can_data, num_states);
        break;
    case 5:
        estimators[num_estimators] = new SwitcherEstimator(config_data, can_data, &tof_sensors[0],num_states);
        break;
    case 6:
        estimators[num_estimators] = new GimbalEstimatorNoOdom(config_data, &buff_sensors[0], &buff_sensors[1], &icm_sensors[0], can_data, num_states);
        break;
    default:
        break;
    }

    num_estimators++;
}

void EstimatorManager::step(float macro_outputs[STATE_LEN][3], float micro_outputs[NUM_MOTORS][MICRO_STATE_LEN], int override) {
    // clear output
    float curr_state[STATE_LEN][3] = {0};
    memcpy(curr_state, macro_outputs, sizeof(curr_state));
    clear_outputs(macro_outputs, micro_outputs);

    for (int i = 0; i < num_estimators; i++) {
        int num_states = estimators[i]->get_num_states();
        float macro_states[STATE_LEN][3] = { 0 };
        float micro_states[NUM_MOTORS][MICRO_STATE_LEN] = { 0 };

        if (!estimators[i]->micro_estimator) {
            
            estimators[i]->step_states(macro_states, curr_state, override);
            for (int j = 0; j < num_states; j++) {
                for (int k = 0; k < 3; k++)
                    macro_outputs[applied_states[i][j]][k] = macro_outputs[applied_states[i][j]][k] + macro_states[j][k];
            }
        } else {
            estimators[i]->step_states(micro_states, curr_state, override);
            for (int j = 0; j < num_states; j++) {
                for (int k = 0; k < MICRO_STATE_LEN; k++) {
                    micro_outputs[applied_states[i][j]][k] = micro_outputs[applied_states[i][j]][k] + micro_states[j][k];
                }
            }
        }
    }
}

void EstimatorManager::clear_outputs(float macro_outputs[STATE_LEN][3], float micro_outputs[NUM_MOTORS][MICRO_STATE_LEN]) {
    for (int i = 0; i < STATE_LEN; i++) {
        for (int j = 0; j < 3; j++) {
            macro_outputs[i][j] = 0;
        }
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        for (int j = 0; j < MICRO_STATE_LEN; j++) {
            micro_outputs[i][j] = 0;
        }
    }
}

void EstimatorManager::assign_states(float as[NUM_ESTIMATORS][STATE_LEN]) {
    for (int i = 0; i < NUM_ESTIMATORS; i++) {
        for (int j = 0; j < STATE_LEN; j++) {
            applied_states[i][j] = (int) as[i][j];
        }
    }
}

void EstimatorManager::read_sensors() {
    buff_sensors[0].read();
    buff_sensors[1].read();
    icm_sensors[0].read();
    rev_sensors[0].read();
    rev_sensors[1].read();
    rev_sensors[2].read();
}

void EstimatorManager::calibrate_imus() {
    Serial.println("Calibrating IMU's...");
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    float sum_accel_x = 0;
    float sum_accel_y = 0;
    float sum_accel_z = 0;
    
    for (int i = 0; i < NUM_IMU_CALIBRATION; i++) {
        icm_sensors[0].read();
        sum_x += icm_sensors[0].get_gyro_X();
        sum_y += icm_sensors[0].get_gyro_Y();
        sum_z += icm_sensors[0].get_gyro_Z();

        sum_accel_x += icm_sensors[0].get_accel_X();
        sum_accel_y += icm_sensors[0].get_accel_Y();
        sum_accel_z += icm_sensors[0].get_accel_Z();
    }

    Serial.printf("Calibrated offsets: %f, %f, %f", sum_x / NUM_IMU_CALIBRATION, sum_y / NUM_IMU_CALIBRATION, sum_z / NUM_IMU_CALIBRATION);
    Serial.println();
    icm_sensors[0].set_offsets(sum_x / NUM_IMU_CALIBRATION, sum_y / NUM_IMU_CALIBRATION, sum_z / NUM_IMU_CALIBRATION);
}


EstimatorManager::~EstimatorManager() {
    Serial.println("Ending SPI");
    SPI.end();
    Serial.println("SPI Ended");

    for (int i = 0; i < STATE_LEN; i++) {
        if (estimators[i] == nullptr)
            continue;
        delete estimators[i];
    }
}