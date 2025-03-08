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

    for (int i = 0; i < NUM_SENSORS; i++) {
        int type = config_data->sensor_info[i][0];
        if (type != -1) {
            num_sensors[type]++;
        }
    }

    // initilize the sensors

    // configure pins for the encoders
    for (int i = 0; i < buff_sensor_count; i++) {
        pinMode(config_data->sensor_info[i][1], OUTPUT);
        digitalWrite(config_data->sensor_info[i][1], HIGH);
        buff_encoders[i] = new BuffEncoder(config_data->sensor_info[i][1]);
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

    }
    calibrate_imus();

    // initialize rev encoders
    for (int i = 0; i < rev_sensor_count; i++) {
        rev_sensors[i] = new RevEncoder();
        rev_sensors[i]->init(REV_ENC_PIN1 + i, true);
    }

    // initialize TOFs
    for (int i = 0; i < tof_sensor_count; i++) {
        tof_sensors[i] = new TOFSensor();
        tof_sensors[i]->init();
    }

    // initialize LiDARs
    if (lidar_sensor_count > 0) {
        lidar1 = new D200LD14P(&Serial4, 0);
        lidar2 = new D200LD14P(&Serial5, 1);

    }


}

void SensorManager::read() {
    for (int i = 0; i < buff_sensor_count; i++) {
        buff_encoders[i]->read();
        buff_encoders[i]->print();
    }
    for (int i = 0; i < icm_sensor_count; i++) {
        icm_sensors[i]->read();
        icm_sensors[i]->print();
    }

    for (int i = 0; i < rev_sensor_count; i++) {
        rev_sensors[i]->read();
        rev_sensors[i]->print();
    }
    if (lidar_sensor_count > 0) {

        lidar1->read();
        lidar2->read();

    }

    // read ref system

    ref->read();
    for (int i = 0; i < tof_sensor_count; i++) {
        tof_sensors[i]->read();
        tof_sensors[i]->print();
    }
}
BuffEncoder* SensorManager::get_buff_encoder(int index) {
    return buff_encoders[index];
}

ICM20649* SensorManager::get_icm_sensor(int index) {
    return icm_sensors[index];
}

RevEncoder* SensorManager::get_rev_sensor(int index) {
    return rev_sensors[index];
}

TOFSensor* SensorManager::get_tof_sensor(int index) {
    return tof_sensors[index];
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

