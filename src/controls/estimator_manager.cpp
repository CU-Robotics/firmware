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

void EstimatorManager::init(CANManager* _can, const Config* _config_data, SensorManager* _sensor_manager) {

    // set can and config data pointers
    can = _can;
    config_data = _config_data;
    sensor_manager = _sensor_manager;
    if (!config_data) Serial.println("CONFIG DATA IS NULL!!!!!");

    // create and initialize the estimators
    for (int i = 0; i < NUM_ESTIMATORS; i++) {
        int id = config_data->estimator_info[i][0];
        // Serial.printf("Init Estimator %d\n", id);

        if (id != -1) {
            init_estimator(id);
        }
    }
}

void EstimatorManager::init_estimator(int estimator_id) {
    if (!config_data) Serial.println("CONFIG DATA IS NULL!!!!!");

    switch (estimator_id) {
    case 1:
        estimators[num_estimators++] = new GimbalEstimator(*config_data, sensor_manager, can);
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
        estimators[num_estimators++] = new SwitcherEstimator(*config_data, can, sensor_manager->get_tof_sensor(0));
        break;
    case 6:
        estimators[num_estimators++] = new GimbalEstimatorNoOdom(*config_data, sensor_manager, can);        
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