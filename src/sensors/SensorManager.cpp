#include "SensorManager.hpp"

SensorManager::SensorManager() {
    return;
};

SensorManager::~SensorManager() {
    delete[] buff_sensors_data;
    delete[] icm_sensors_data;
    delete[] rev_sensors_data;
    delete[] tof_sensors_data;
    delete[] lidar_sensors_data;
}

void SensorManager::init(const Config* config_data) {
    buff_sensor_count = config_data->num_of_buffEnc;
    icm_sensor_count = config_data->num_of_icm;
    rev_sensor_count = config_data->num_of_revEnc;
    tof_sensor_count = config_data->num_of_tof;
    lidar_sensor_count = config_data->num_of_lidar;

    // Allocate memory for sensor arrays
    buff_sensors_data = new BuffEncoderData[buff_sensor_count];
    icm_sensors_data = new ICMSensorData[icm_sensor_count];
    rev_sensors_data = new RevSensorData[rev_sensor_count];
    tof_sensors_data = new TOFSensorData[tof_sensor_count];
    lidar_sensors_data = new LidarSensorData[lidar_sensor_count];

    for(int i = 0; i < NUM_SENSORS; i++){
        int type = config_data->sensor_info[i][0];
        if(type != -1){
            num_sensors[type]++;
        }

        //Split up the sensor counts by type
        switch (type)
        {
        case SensorType::BUFFENC:
            buff_sensor_count++;
            break;
        case SensorType::REVENC:
            rev_sensor_count++;
            break;
        case SensorType::ICM:
            icm_sensor_count++;
            break;
        case SensorType::TOF:
            tof_sensor_count++;
            break;
        case SensorType::LIDAR:
            lidar_sensor_count++;
            break;
        default:
            break;
        }
    }

    //initilize the sensors

    
    // configure pins for the encoders
    for (int i = 0;i < num_sensors[0];i++) {
        pinMode(config_data->sensor_info[i][1], OUTPUT);
        digitalWrite(config_data->sensor_info[i][1], HIGH);
    }
     // initialize buff encoders
    for (int i = 0;i < num_sensors[0];i++) {
        buff_encoders[i].init(config_data->sensor_info[i][1]);
    }

    // configure pins for the ICM
    pinMode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH);

    // start SPI
    Serial.println("Starting SPI");
    SPI.begin();

    // initialize ICMs
    for (int i = 0; i < num_sensors[2];i++) {
        icm_sensors[i].init(icm_sensors[i].CommunicationProtocol::SPI);
        icm_sensors[i].set_gyro_range(4000);
    }
    calibrate_imus();

    // initialize rev encoders
    for (int i = 0;i < num_sensors[1];i++) {
        rev_sensors[i].init(REV_ENC_PIN1 + i, true);
    }

    // initialize TOFs
    for (int i = 0;i < num_sensors[3];i++) {
        tof_sensors[i].init();
    }


}

void SensorManager::read() {
    for(int i = 0; i < buff_sensor_count; i++) {
        buff_encoders[i].read();
    }
    for(int i = 0; i < icm_sensor_count; i++) {
        icm_sensors[i].read();
    }
    for(int i = 0; i < rev_sensor_count; i++) {
        rev_sensors[i].read();
    }
    for(int i = 0; i < tof_sensor_count; i++) {
        tof_sensors[i].read();
    }
    
}

/// @brief get the specified buff encoder sensor from the array
    /// @param index index of the sensor object to get
    /// @return reference to the buff encoder sensor
    BuffEncoder* SensorManager::get_buff_encoder(int index) {
        return &buff_encoders[index];

    }

    /// @brief get the specified icm sensor from the array
    /// @param index index of the sensor object to get
    /// @return reference to the icm sensor
    ICM20649* SensorManager::get_icm_sensor(int index) {
        return &icm_sensors[index];
    }

    /// @brief get the specified rev sensor from the array
    /// @param index index of the sensor object to get
    /// @return reference to the rev sensor
    RevEncoder* SensorManager::get_rev_sensor(int index) {
        return &rev_sensors[index];
    }

    /// @brief get the specified tof sensor from the array
    /// @param index index of the sensor object to get
    /// @return reference to the tof sensor
    TOFSensor* SensorManager::get_tof_sensor(int index) {
        return &tof_sensors[index];
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