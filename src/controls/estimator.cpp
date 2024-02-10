#include "estimator.hpp"

void EstimatorManager::init_estimator(int state_id){
    switch(state_id){
        case 4:
            float values[8];
            values[1] = 0;
            estimators[4] = new PitchEstimator(values, buff_sensors[1], can);
            break;
        default:
            estimators[state_id] = new NullEstimator();
    }
}

void EstimatorManager::step(float outputs[STATE_LEN][3]){
    for (int i = 0; i < STATE_LEN; i++) {
        output[i][0] = estimators[i].step_position();
        output[i][1] = estimators[i].step_velocity();
        output[i][2] = estimators[i].step_acceleration();
    }

    memcpy(this->output, outputs, STATE_LEN * 3 * sizeof(float))
}

void EstimatorManager::init(DR16 dr, rm_CAN rm_can){
    pinMode(YAW_BUFF_CS, OUTPUT);
    pinMode(PITCH_BUFF_CS, OUTPUT);
    digitalWrite(YAW_BUFF_CS, HIGH);
    digitalWrite(PITCH_BUFF_CS, HIGH);
    Serial.println("Starting SPI");
    SPI.begin();
    Serial.println("SPI Started");


    buff_sensors[0] = BuffEncoder(YAW_BUFF_CS);
    buff_sensors[1] = BuffEncoder(PITCH_BUFF_CS);

    icm_sensors[0].init(icm[0].CommunicationProtocol::SPI);

    dr16 = dr16;
    can = rm_can;
}

void EstimatorManager::read_sensors(){
    buff_sensors[0].read();
    buff_sensors[1].read();
    icm_sensors[0].read();
}