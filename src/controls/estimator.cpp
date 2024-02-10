#include "estimator.hpp"

void EstimatorManager::init_estimator(int state_id){
    switch(state_id){
        case 4:
            float values[8];
            values[1] = 0; //pitch offset
            estimators[4] = new PitchEstimator(values, buff_sensors[1], &can);
            break;
        default:
            break;
    }
}

void EstimatorManager::step(float outputs[STATE_LEN][3]){
    for (int i = 0; i < STATE_LEN; i++) {
        if(estimators[i] == nullptr) continue;
        output[i][0] = estimators[i]->step_position();
        output[i][1] = estimators[i]->step_velocity();
        output[i][2] = estimators[i]->step_acceleration();
    }

    memcpy(outputs, this->output, STATE_LEN * 3 * sizeof(float));
}

void EstimatorManager::init(){

    // icm_sensors[0].init(icm_sensors[0].CommunicationProtocol::SPI);

    pinMode(YAW_BUFF_CS, OUTPUT);
    pinMode(PITCH_BUFF_CS, OUTPUT);
    pinMode(ICM_CS, OUTPUT);
    digitalWrite(YAW_BUFF_CS, HIGH);
    digitalWrite(PITCH_BUFF_CS, HIGH);
    digitalWrite(ICM_CS, HIGH);
    Serial.println("Starting SPI");
    SPI.begin();
    Serial.println("SPI Started");


    buff_sensors[0] = BuffEncoder(YAW_BUFF_CS);
    buff_sensors[1] = BuffEncoder(PITCH_BUFF_CS);
    Serial.print("beans1 ");

    can.init();
    Serial.print("beans2");
}

void EstimatorManager::read_sensors(){
    buff_sensors[0].read();
    buff_sensors[1].read();
    can.read();
    Serial.print("test");
    // icm_sensors[0].read();
}

rm_CAN* EstimatorManager::get_can(){    
    return &can;

}