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

void EstimatorManager::init_estimator(int state_id) {
    switch (state_id)
    {
    case 4:
        float values[8];
        values[1] = 1; //pitch offset
        estimators[4] = new PitchEstimator(values, &buff_sensors[1], &icm_sensors[0], can_data);
        break;
    default:
        break;
    }
}

void EstimatorManager::step(float outputs[STATE_LEN][3]) {

    for (int i = 0; i < STATE_LEN; i++)
    {
        if (estimators[i] == nullptr) continue;
        Serial.printf("step i=%d %p\n", i, estimators[i]);
        outputs[i][0] = estimators[i]->step_position();
        outputs[i][1] = estimators[i]->step_velocity();
        outputs[i][2] = estimators[i]->step_acceleration();
    }
}

void EstimatorManager::read_sensors()
{
    buff_sensors[0].read();
    buff_sensors[1].read();
    Serial.print("test");
    icm_sensors[0].read();
}

EstimatorManager::~EstimatorManager(){
    for(int i = 0; i < STATE_LEN;i++) {
        if(estimators[i] == nullptr) continue;
        delete estimators[i];
    }
}