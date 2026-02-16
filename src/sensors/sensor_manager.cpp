#include "SensorManager.hpp"
#include "buff_encoder.hpp"

SensorManager::SensorManager() {
     // initialize refereree system
    ref = new RefSystem();
    ref->init();
};

SensorManager::~SensorManager() {}

void SensorManager::init() {

    for (int i = 0; i < NUM_SENSORS; i++) {
        int type = config_data->sensor_info[i][0];
        if (type != -1) {
            num_sensors[type]++;
        }
    }
    for(const auto& buff_encoder_config : config_data->buff_encoders) {
        buff_encoders.push_back(BuffEncoder(buff_encoder_config));
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
    // for (int i = 0; i < tof_sensor_count; i++) {
    //     tof_sensors[i] = new TOFSensor();
    //     tof_sensors[i]->init();

    //     //assign id for comms
    //     tof_sensors[i]->setId(i);
    // }

    // initialize LiDARs
    if (lidar_sensor_count > 0) {
        lidar1 = new D200LD14P(&Serial4, 0);
        lidar2 = new D200LD14P(&Serial5, 1);
    }

    // initialize limit switchesestimated_state
    int limit_switch_index = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (config_data->sensor_info[i][0] != 6) continue;
        limit_switches[limit_switch_index++] = new LimitSwitch(config_data->sensor_info[i][1]);
    }
}

void SensorManager::configure(const NewConfig::RobotConfig& config_data) {
    for (const auto& buff_encoder_config : config_data.buff_encoders) {
        buff_encoders.insert(BuffEncoder(buff_encoder_config), Comms::Sendable<BuffEncoderData>());
    }

    for (const auto& rev_encoder_config : config_data.rev_encoders) {
        rev_encoders.insert(RevEncoder(rev_encoder_config), Comms::Sendable<RevEncoderData>());
    }

    for (const auto& icm_config : config_data.icms) {
        icm_imus.insert(ICM20649(icm_config), Comms::Sendable<ICMData>());
    }

    for (const auto& lsm_config : config_data.lsm_imus) {
        lsm_imus.insert(LSM6DSOX(lsm_config), Comms::Sendable<LSMData());
    }

    for (const auto& d200_lidar_config : config_data.d200_lidars) {
        d200_lidars.insert(D200LD14P(d200_lidar_config), Comms::Sendable<LidarDataPacketSI>());
    }

    for (const auto& limit_switch_config : config_data.limit_switches) {
        limit_switches.insert(LimitSwitch(limit_switch_config), Comms::Sendable<LimitSwitchData>());
    }

    for (const auto& current_sensor_config : config_data.acs712_current_sensors) {
        acs712_current_sensors.insert(ACS712(current_sensor_config), Comms::Sendable<ACS712Data>());
    }

    for (const auto& tof_config : config_data.tof_sensors) {
        tof_sensors.insert(TOFSensor(tof_config), Comms::Sendable<TOFData>());
    }

    for (const auto& stereo_cam_trigger_config : config_data.stereo_cam_triggers) {
        stereo_cam_triggers.insert(StereoCamTrigger(stereo_cam_trigger_config), Comms::Sendable<StereoCamTriggerData());
    }
}

void SensorManager::read_and_send_to_comms() {
    for (auto& [buff_enc, buff_enc_sendable] : buff_encoders) {
        buff_enc.read();
        
        buff_enc_sendable.data = buff_enc.get_data_for_comms();
        buff_enc_sendable.send_to_comms();
    }

    for (auto& [rev_enc, rev_encoder_sendable] : rev_encoders) {
        rev_enc.read();

        rev_encoder_sendable.data = rev_enc.get_data_for_comms();
        rev_encoder_sendable.send_to_comms();
    }

    for (auto& [icm_imu, icm_sendable] : icm_imus) {
        icm_imu.read();
    }

    for (auto& [lsm_imu, lsm_sendable] : lsm_imus) {
        lsm_imu.read();
    }

    for (auto& [d200_lidar, lidar_sendable] : d200_lidars) {
        d200_lidar.read();
    }

    for (auto& [limit_switch, limit_switch_sendable] : limit_switches) {
        limit_switch.read();
    }

    for (auto& [acs712_current_sensor, acs712_sendable] : acs712_current_sensors) {
        acs712_current_sensor.read();
    }

    for (auto& [tof_sensor, tof_sendable] : tof_sensors) {
        tof_sensor.read();
    }

    //Stereo Cam Trigger doesn't need to be read from, it updates its data in the timer interrupt callback

    //NEED TO FIGURE OUT SYNCING GIMBAL YAW WITH LIDAR AND STEREO CAM READINGS

}


std::optional<BuffEncoder*> SensorManager::get_buff_encoder_by_name(NewConfig::BuffEncoderName name) {
    for (auto& [buff_enc, buff_enc_sendable] : buff_encoders) {
        if (buff_enc.get_name() == name) {
            return &buff_enc;
        }
    }
    return std::nullopt;
}

std::optional<RevEncoder*> SensorManager::get_rev_encoder_by_name(NewConfig::RevEncoderName name) {
    for (auto& [rev_enc, rev_enc_sendable] : rev_encoders) {
        if (rev_enc.get_name() == name) {
            return &rev_enc;
        }
    }
    return std::nullopt;
}

std::optional<ICM20649*> SensorManager::get_icm_sensor_by_name(NewConfig::ImuName name) {
    for (auto& [icm_imu, icm_sendable] : icm_imus) {
        if (icm_imu.get_name() == name) {
            return &icm_imu;
        }
    }
    return std::nullopt;
}

std::optional<LSM6DSOX*> SensorManager::get_lsm_sensor_by_name(NewConfig::ImuName name) {
    for (auto& [lsm_imu, lsm_sendable] : lsm_imus) {
        if (lsm_imu.get_name() == name) {
            return &lsm_imu;
        }
    }
    return std::nullopt;
}

std::optional<D200LD14P*> SensorManager::get_lidar_sensor_by_name(NewConfig::D200LidarName name) {
    for (auto& [d200_lidar, lidar_sendable] : d200_lidars) {
        if (d200_lidar.get_name() == name) {
            return &d200_lidar;
        }
    }
    return std::nullopt;
}

std::optional<LimitSwitch*> SensorManager::get_limit_switch_by_name(NewConfig::LimitSwitchName name) {
    for (auto& [limit_switch, limit_switch_sendable] : limit_switches) {
        if (limit_switch.get_name() == name) {
            return &limit_switch;
        }
    }
    return std::nullopt;
}

std::optional<ACS712*> SensorManager::get_current_sensor_by_name(NewConfig::CurrentSensorName name) {
    for (auto& [current_sensor, current_sensor_sendable] : acs712_current_sensors) {
        if (current_sensor.get_name() == name) {
            return &current_sensor;
        }
    }
    return std::nullopt;
}

std::optional<TOFSensor*> SensorManager::get_tof_sensor_by_name(NewConfig::TOFSensorName name) {
    for (auto& [tof_sensor, tof_sendable] : tof_sensors) {
        if (tof_sensor.get_name() == name) {
            return &tof_sensor;
        }
    }
    return std::nullopt;
}

std::optional<StereoCamTrigger*> SensorManager::get_stereo_cam_trigger_by_name(NewConfig::StereoCameraTriggerName name) {
    for (auto& [stereo_cam_trigger, stereo_cam_trigger_sendable] : stereo_cam_triggers) {
        if (stereo_cam_trigger.get_name() == name) {
            return &stereo_cam_trigger;
        }
    }
    return std::nullopt;
}

void SensorManager::calibrate_imus() {
    Serial.println("Calibrating IMU's...");
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    for (int i = 0; i < NUM_IMU_CALIBRATION; i++) {
        icm_sensors[0]->read();
        sum_x += icm_sensors[0]->get_gyro_X();
        sum_y += icm_sensors[0]->get_gyro_Y();
        sum_z += icm_sensors[0]->get_gyro_Z();
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
    // for(int i = 0; i < tof_sensor_count; i++) {
    //     tof_sensor_sendables[i].data.latest_distance = tof_sensors[i]->get_latest_distance();
    //     tof_sensor_sendables[i].data.id = tof_sensors[i]->getId();
    //     tof_sensor_sendables[i].send_to_comms();
    // }

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