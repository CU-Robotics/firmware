#include "SensorManager.hpp"
#include "../utils/profiler.hpp"

SensorManager::SensorManager() {
     // initialize refereree system
    ref = new RefSystem();
    ref->init();
};

SensorManager::~SensorManager() {
}

void SensorManager::init(const Config* config_data) {
    buff_sensor_count = config_data->num_of_buffEnc;
    icm_sensor_count = config_data->num_of_icm;
    rev_sensor_count = config_data->num_of_revEnc;
    tof_sensor_count = config_data->num_of_tof;
    lidar_sensor_count = config_data->num_of_lidar;
    limit_switch_count = config_data->num_of_limit_switch;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int type = config_data->sensor_info[i][0];
        if (type != -1) {
            num_sensors[type]++;
        }
    }

    // initilize the sensors
    // configure pins for the encoders
    int buff_enc_index = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (config_data->sensor_info[i][0] != 0) continue;
        pinMode(config_data->sensor_info[i][1], OUTPUT);
        digitalWrite(config_data->sensor_info[i][1], HIGH);
        buff_encoders[buff_enc_index] = new BuffEncoder(config_data->sensor_info[i][1]);

        //assign id for comms
        buff_encoders[buff_enc_index]->setId(buff_enc_index);
        buff_enc_index++;
    }

    // configure pins for the ICM
    pinMode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH);

    // start SPI
    Serial.println("Starting SPI");
    SPI.begin();

    // initialize ICMs
    for (int i = 0; i < icm_sensor_count; i++) {
        icm_sensors[i] = new ICM20649();
        icm_sensors[i]->init(icm_sensors[i]->CommunicationProtocol::SPI);
        icm_sensors[i]->set_gyro_range(4000);

        //assign id for comms
        icm_sensors[i]->setId(i);
    }
    if(icm_sensor_count > 0) calibrate_imus();

    // initialize rev encoders
    for (int i = 0; i < rev_sensor_count; i++) {
        rev_sensors[i].init(REV_ENC_PIN1 + i, true);

        //assign id for comms
        rev_sensors[i].setId(i);
    }

    // initialize TOFs
    for (int i = 0; i < tof_sensor_count; i++) {
        tof_sensors[i] = new TOFSensor();
        tof_sensors[i]->init();

        //assign id for comms
        tof_sensors[i]->setId(i);
    }

    // initialize LiDARs
    if (lidar_sensor_count > 0) {
        lidar1 = new D200LD14P(&Serial4, 0);
        lidar2 = new D200LD14P(&Serial5, 1);
    }

    // initialize limit switches
    int limit_switch_index = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (config_data->sensor_info[i][0] != 6) continue;
        limit_switches[limit_switch_index++] = new LimitSwitch(config_data->sensor_info[i][1]);
    }
}

void SensorManager::read() {
    for (int i = 0; i < buff_sensor_count; i++) {
        buff_encoders[i]->read();
    }
    for (int i = 0; i < icm_sensor_count; i++) {
        icm_sensors[i]->read();
    }
    for (int i = 0; i < rev_sensor_count; i++) {
        rev_sensors[i].read();
    }
    if (lidar_sensor_count > 0) {
        lidar1->set_yaw(estimated_state[3][0], estimated_state[3][1]);
        lidar1->read();

        lidar2->set_yaw(estimated_state[3][0], estimated_state[3][1]);
        lidar2->read();
    }
    // read ref system
    ref->read();

    // read tof sensors
    for (int i = 0; i < tof_sensor_count; i++) {
        // tof_sensors[i]->read();
        // tof_sensors[i]->print();
    }
}
BuffEncoder* SensorManager::get_buff_encoder(int index) {
    return buff_encoders[index];
}

ICM20649* SensorManager::get_icm_sensor(int index) {
    return icm_sensors[index];
}

RevEncoder* SensorManager::get_rev_sensor(int index) {
    return &rev_sensors[index];
}

TOFSensor* SensorManager::get_tof_sensor(int index) {
    return tof_sensors[index];
}

LimitSwitch* SensorManager::get_limit_switch(int index) {
    return limit_switches[index];
}


D200LD14P* SensorManager::get_lidar_sensor(int index) {
    if (index == 0) {
        return lidar1;
    } else if (index == 1) {
        return lidar2;
    } else {
        return nullptr;
    }
}

void SensorManager::calibrate_imus() {
    Serial.println("Calibrating IMU's...");
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    float sum_accel_x = 0;
    float sum_accel_y = 0;
    float sum_accel_z = 0;

    for (int i = 0; i < NUM_IMU_CALIBRATION; i++) {
        icm_sensors[0]->read();
        sum_x += icm_sensors[0]->get_gyro_X();
        sum_y += icm_sensors[0]->get_gyro_Y();
        sum_z += icm_sensors[0]->get_gyro_Z();

        sum_accel_x += icm_sensors[0]->get_accel_X();
        sum_accel_y += icm_sensors[0]->get_accel_Y();
        sum_accel_z += icm_sensors[0]->get_accel_Z();
    }

    Serial.printf("Calibrated offsets: %f, %f, %f\n", sum_x / NUM_IMU_CALIBRATION, sum_y / NUM_IMU_CALIBRATION, sum_z / NUM_IMU_CALIBRATION);
    icm_sensors[0]->set_offsets(sum_x / NUM_IMU_CALIBRATION, sum_y / NUM_IMU_CALIBRATION, sum_z / NUM_IMU_CALIBRATION);
}

void SensorManager::send_sensor_data_to_comms()
{
    //send sensor data to comms

    //send buff encoders
    for(int i = 0; i < buff_sensor_count; i++) {
        buff_encoder_sendables[i].data.m_angle = buff_encoders[i]->get_angle();
        buff_encoder_sendables[i].data.id = buff_encoders[i]->getId();
        buff_encoder_sendables[i].send_to_comms();
    }

    //send ICMs
    for(int i = 0; i < icm_sensor_count; i++) {
        icm_sendables[i].data.accel_X = icm_sensors[i]->get_accel_X();
        icm_sendables[i].data.accel_Y = icm_sensors[i]->get_accel_Y();
        icm_sendables[i].data.accel_Z = icm_sensors[i]->get_accel_Z();
        icm_sendables[i].data.gyro_X = icm_sensors[i]->get_gyro_X();
        icm_sendables[i].data.gyro_Y = icm_sensors[i]->get_gyro_Y();
        icm_sendables[i].data.gyro_Z = icm_sensors[i]->get_gyro_Z();
        icm_sendables[i].data.temperature = icm_sensors[i]->get_temperature();
        icm_sendables[i].data.id = icm_sensors[i]->getId();
        icm_sendables[i].send_to_comms();
    }

    //send rev encoders
    for(int i = 0; i < rev_sensor_count; i++) {
        rev_sensor_sendables[i].data.ticks = rev_sensors[i].get_angle_ticks();
        rev_sensor_sendables[i].data.radians = rev_sensors[i].get_angle_radians();
        rev_sensor_sendables[i].data.id = rev_sensors[i].getId();
        rev_sensor_sendables[i].send_to_comms();
    }

    //send TOFs
    for(int i = 0; i < tof_sensor_count; i++) {
        tof_sensor_sendables[i].data.latest_distance = tof_sensors[i]->get_latest_distance();
        tof_sensor_sendables[i].data.id = tof_sensors[i]->getId();
        tof_sensor_sendables[i].send_to_comms();
    }

    //send LiDARs
    //NOTE it seems like lidar is broken right now
    if(lidar_sensor_count > 0) {
        LidarDataPacketSI lidar_data[2] = {};
        lidar1->get_data(lidar_data);

        for (int i = 0; i < 2; i++) {
            lidar_sensor_sendables[i] = lidar_data[i];
            lidar_sensor_sendables[i].data.id = 0;
            lidar_sensor_sendables[i].send_to_comms();
        }

        lidar2->get_data(lidar_data);

        for (int i = 0; i < 2; i++) {
            lidar_sensor_sendables[i] = lidar_data[i];
            lidar_sensor_sendables[i].data.id = 1;
            lidar_sensor_sendables[i].send_to_comms();
        }
    }

}

int SensorManager::get_num_sensors(SensorType sensor_type) {
    switch (sensor_type) {
    case SensorType::BUFFENC:
        return buff_sensor_count;
    case SensorType::ICM:
        return icm_sensor_count;
    case SensorType::REVENC:
        return rev_sensor_count;
    case SensorType::TOF:
        return tof_sensor_count;
    case SensorType::LIDAR:
        return lidar_sensor_count;
    default:
        return 0;
    }
}

