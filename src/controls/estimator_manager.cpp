#include "estimator_manager.hpp"

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

void EstimatorManager::init(CANManager* _can, const Config* _config_data) {

    // set can and config data pointers
    can = _can;
    config_data = _config_data;
    if (!config_data) Serial.println("CONFIG DATA IS NULL!!!!!");

    for(int i = 0; i < NUM_SENSORS; i++){
        int type = config_data->sensor_info[i][0];
        if(type != -1){
            num_sensors[type]++;
        }
    }

    // configure pins for the encoders
    for (int i = 0;i < num_sensors[0];i++) {
        pinMode(config_data->sensor_info[i][1], OUTPUT);
        digitalWrite(config_data->sensor_info[i][1], HIGH);
    }

    // configure pins for the ICM
    pinMode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH);

    // start SPI
    Serial.println("Starting SPI");
    SPI.begin();

    // initialize buff encoders
    for (int i = 0;i < num_sensors[0];i++) {
        buff_encoders[i].init(config_data->sensor_info[i][1]);
    }

    // initialize ICMs
    for (int i = 0; i < num_sensors[2];i++) {
        icm_sensors[i].init(icm_sensors[i].CommunicationProtocol::SPI);
        icm_sensors[i].set_gyro_range(4000);
    }

    // initialize rev encoders
    for (int i = 0;i < num_sensors[1];i++) {
        rev_sensors[i].init(REV_ENC_PIN1 + i, true);
    }

    // initialize TOFs
    for (int i = 0;i < num_sensors[3];i++) {
        tof_sensors[i].init();
    }

    // create and initialize the estimators
    for (int i = 0; i < NUM_ESTIMATORS; i++) {
        int id = config_data->estimator_info[i][0];
        // Serial.printf("Init Estimator %d\n", id);

        if (id != -1) {
            init_estimator(id);
        }
    }

    // calibrate the IMUs
    calibrate_imus();
}

void EstimatorManager::init_estimator(int estimator_id) {
    if (!config_data) Serial.println("CONFIG DATA IS NULL!!!!!");

    switch (estimator_id) {
    case 1:
        estimators[num_estimators++] = new GimbalEstimator(*config_data, &rev_sensors[0], &rev_sensors[1], &rev_sensors[2], &buff_encoders[0], &buff_encoders[1], &icm_sensors[0], can);
        break;
    case 2:
        estimators[num_estimators++] = new FlyWheelEstimator(can);
        break;
    case 3:
        estimators[num_estimators++] = new FeederEstimator(can);
        break;
    case 4:
        estimators[num_estimators++] = new LocalEstimator(can);
        break;
    case 5:
        estimators[num_estimators++] = new SwitcherEstimator(*config_data, can, &tof_sensors[0]);
        break;
    case 6:
        estimators[num_estimators++] = new GimbalEstimatorNoOdom(*config_data, &buff_encoders[0], &buff_encoders[1], &icm_sensors[0], can);
        break;
    default:
        break;
    }
}

void EstimatorManager::step(float macro_outputs[STATE_LEN][3], float micro_outputs[CAN_MAX_MOTORS][MICRO_STATE_LEN], int override) {
    // clear output
    float curr_state[STATE_LEN][3] = { 0 };
    memcpy(curr_state, macro_outputs, sizeof(curr_state));
    clear_outputs(macro_outputs, micro_outputs);


    for (int i = 0; i < num_estimators; i++) {
        float macro_states[STATE_LEN][3] = { 0 };
        float micro_states[CAN_MAX_MOTORS][MICRO_STATE_LEN] = { 0 };

        if (!estimators[i]->micro_estimator) {

            estimators[i]->step_states(macro_states, curr_state, override);

            for (int j = 0; j < STATE_LEN + 1; j++) {
                int index = config_data->estimator_info[i][j + 1]; // j + 1 because the id is in index 0
                if(index == -1) break;
                for (int k = 0; k < 3; k++){
                    macro_outputs[index][k] = macro_states[j][k];
                }
            }
        } else {
            estimators[i]->step_states(micro_states, curr_state, override);
            for (size_t j = 0; j < CAN_MAX_MOTORS + 1; j++) {
                int index = config_data->estimator_info[i][j + 1]; //0 index is reserved for the id
                if (index == -1) break;
                for (int k = 0; k < MICRO_STATE_LEN; k++) {
                    micro_outputs[index][k] = micro_states[j][k];
                }
            }
        }
    }
}

void EstimatorManager::clear_outputs(float macro_outputs[STATE_LEN][3], float micro_outputs[CAN_MAX_MOTORS][MICRO_STATE_LEN]) {
    for (int i = 0; i < STATE_LEN; i++) {
        for (int j = 0; j < 3; j++) {
            macro_outputs[i][j] = 0;
        }
    }
    for (size_t i = 0; i < CAN_MAX_MOTORS; i++) {
        for (int j = 0; j < MICRO_STATE_LEN; j++) {
            micro_outputs[i][j] = 0;
        }
    }
}


void EstimatorManager::read_sensors() {
    if (!config_data)
        Serial.println("CONFIG DATA IS NULL!!!!!");

    for (int i = 0; i < num_sensors[0]; i++) {
        buff_encoders[i].read();
        buff_encoders[i].print();
    }
    for (int i = 0; i < num_sensors[2]; i++) {
        icm_sensors[i].read();
        icm_sensors[i].print();
    }
    for (int i = 0; i < num_sensors[1]; i++) {
        rev_sensors[i].read();
        rev_sensors[i].print();
    }
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

    Serial.printf("Calibrated offsets: %f, %f, %f\n", sum_x / NUM_IMU_CALIBRATION, sum_y / NUM_IMU_CALIBRATION, sum_z / NUM_IMU_CALIBRATION);
    icm_sensors[0].set_offsets(sum_x / NUM_IMU_CALIBRATION, sum_y / NUM_IMU_CALIBRATION, sum_z / NUM_IMU_CALIBRATION);
}