#include "SensorManager.hpp"
#include "buff_encoder.hpp"

SensorManager::SensorManager() {}

SensorManager::~SensorManager() {}

void SensorManager::init(const NewConfig::RobotConfig& config_data) {

    // start SPI
    Serial.println("Starting SPI");
    SPI.begin();

    configure_sensors(config_data);

    initialize_sensors();
}

void SensorManager::configure_sensors(const NewConfig::RobotConfig& config_data) {
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

void SensorManager::initialize_sensors(){
    for (auto& [buff_enc, buff_enc_sendable] : buff_encoders) {
        buff_enc.init();
    }

    for (auto& [rev_enc, rev_encoder_sendable] : rev_encoders) {
        rev_enc.init();
    }

    for (auto& [icm_imu, icm_sendable] : icm_imus) {
        icm_imu.init();
    }

    for (auto& [lsm_imu, lsm_sendable] : lsm_imus) {
        lsm_imu.init();
    }

    for (auto& [d200_lidar, lidar_sendable] : d200_lidars) {
        d200_lidar.init();
    }

    for (auto& [limit_switch, limit_switch_sendable] : limit_switches) {
        limit_switch.init();
    }

    for (auto& [acs712_current_sensor, acs712_sendable] : acs712_current_sensors) {
        acs712_current_sensor.init();
    }

    for (auto& [tof_sensor, tof_sendable] : tof_sensors) {
        tof_sensor.init();
    }

    for (auto& [stereo_cam_trigger, stereo_cam_trigger_sendable] : stereo_cam_triggers) {
        stereo_cam_trigger.init();
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

        icm_sendable.data = icm_imu.get_data_for_comms();
        icm_sendable.send_to_comms();
    }

    for (auto& [lsm_imu, lsm_sendable] : lsm_imus) {
        lsm_imu.read();

        lsm_sendable.data = lsm_imu.get_data_for_comms();
        lsm_sendable.send_to_comms();
    }

    for (auto& [d200_lidar, lidar_sendable] : d200_lidars) {
        d200_lidar.read();

        lidar_sendable.data = d200_lidar.get_data_for_comms();
        lidar_sendable.send_to_comms();
    }

    for (auto& [limit_switch, limit_switch_sendable] : limit_switches) {
        limit_switch.read();

        limit_switch_sendable.data = limit_switch.get_data_for_comms();
        limit_switch_sendable.send_to_comms();
    }

    for (auto& [acs712_current_sensor, acs712_sendable] : acs712_current_sensors) {
        acs712_current_sensor.read();

        acs712_sendable.data = acs712_current_sensor.get_data_for_comms();
        acs712_sendable.send_to_comms();
    }

    for (auto& [tof_sensor, tof_sendable] : tof_sensors) {
        tof_sensor.read();

        tof_sendable.data = tof_sensor.get_data_for_comms();
        tof_sendable.send_to_comms();
    }

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