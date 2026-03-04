#include "ACS712.hpp"
#include "ICM20649.hpp"
#include "SensorManager.hpp"
#include "buff_encoder.hpp"

SensorManager::SensorManager() {}

SensorManager::~SensorManager() {
    Serial.println("Ending SPI");
    SPI.end();
    Serial.println("SPI Ended");
}

void SensorManager::init(const NewConfig::RobotConfig& config_data) {
    // start SPI
    Serial.println("Starting SPI");
    SPI.begin();
    Serial.println("SPI Started");

    configure_sensors(config_data);

    initialize_sensors();
}

void SensorManager::configure_sensors(const NewConfig::RobotConfig& config_data) {
    for (const auto& buff_encoder_config : config_data.buff_encoders) {
        buff_encoders.emplace(buff_encoder_config.name, BuffEncoder(buff_encoder_config));
    }

    for (const auto& rev_encoder_config : config_data.rev_encoders) {
        rev_encoders.emplace(rev_encoder_config.name, RevEncoder(rev_encoder_config));
    }

    for (const auto& icm_config : config_data.icms) {
        icm_imus.emplace(icm_config.name, ICM20649(icm_config));
    }

    for (const auto& lsm_config : config_data.lsm_imus) {
        lsm_imus.emplace(lsm_config.name, LSM6DSOX(lsm_config));
    }

    for (const auto& d200_lidar_config : config_data.d200_lidars) {
        d200_lidars.emplace(d200_lidar_config.name, D200LD14P(d200_lidar_config));
    }

    for (const auto& limit_switch_config : config_data.limit_switches) {
        limit_switches.emplace(limit_switch_config.name, LimitSwitch(limit_switch_config));
    }

    for (const auto& current_sensor_config : config_data.acs712_current_sensors) {
        acs712_current_sensors.emplace(current_sensor_config.name, ACS712(current_sensor_config));
    }

    for (const auto& tof_config : config_data.tof_sensors) {
        tof_sensors.emplace(tof_config.name, TOFSensor(tof_config));
    }

    for (const auto& stereo_cam_trigger_config : config_data.stereo_cam_triggers) {
        stereo_cam_triggers.emplace(stereo_cam_trigger_config.name, StereoCamTrigger(stereo_cam_trigger_config));
    }
}

void SensorManager::initialize_sensors(){
    for (auto& [buff_enc_name, buff_enc] : buff_encoders) {
        buff_enc.init();
    }

    for (auto& [rev_enc_name, rev_enc] : rev_encoders) {
        rev_enc.init();
    }

    for (auto& [icm_imu_name, icm_imu] : icm_imus) {
        icm_imu.init();
    }

    for (auto& [lsm_imu_name, lsm_imu] : lsm_imus) {
        lsm_imu.init();
    }

    for (auto& [d200_lidar_name, d200_lidar] : d200_lidars) {
        d200_lidar.init();
    }

    for (auto& [limit_switch_name, limit_switch] : limit_switches) {
        limit_switch.init();
    }

    for (auto& [acs712_current_sensor_name, acs712_current_sensor] : acs712_current_sensors) {
        acs712_current_sensor.init();
    }

    for (auto& [tof_sensor_name, tof_sensor] : tof_sensors) {
        tof_sensor.init();
    }

    for (auto& [stereo_cam_trigger_name, stereo_cam_trigger] : stereo_cam_triggers) {
        stereo_cam_trigger.init();
    }
}

void SensorManager::read() {
    for (auto& [buff_enc_name, buff_enc] : buff_encoders) {
        buff_enc.read();
    }

    for (auto& [rev_enc_name, rev_enc] : rev_encoders) {
        rev_enc.read();
    }

    for (auto& [icm_imu_name, icm_imu] : icm_imus) {
        icm_imu.read();
    }

    for (auto& [lsm_imu_name, lsm_imu] : lsm_imus) {
        lsm_imu.read();
    }

    for (auto& [d200_lidar_name, d200_lidar] : d200_lidars) {
        d200_lidar.read();
    }

    for (auto& [limit_switch_name, limit_switch] : limit_switches) {
        limit_switch.read();
    }

    for (auto& [acs712_current_sensor_name, acs712_current_sensor] : acs712_current_sensors) {
        acs712_current_sensor.read();
    }

    for (auto& [tof_sensor_name, tof_sensor] : tof_sensors) {
        tof_sensor.read();
    }

    //NEED TO FIGURE OUT SYNCING GIMBAL YAW WITH LIDAR AND STEREO CAM READINGS
}

void SensorManager::send_to_comms() {
    for (auto& [buff_enc_name, buff_enc] : buff_encoders) {
        Comms::Sendable<BuffEncoderData> buff_encoder_sendable;
        buff_encoder_sendable.data = buff_enc.get_data_for_comms();
        buff_encoder_sendable.send_to_comms();
    }

    for (auto& [rev_enc_name, rev_enc] : rev_encoders) {
        Comms::Sendable<RevEncoderData> rev_encoder_sendable;
        rev_encoder_sendable.data = rev_enc.get_data_for_comms();
        rev_encoder_sendable.send_to_comms();
    }

    for (auto& [icm_imu_name, icm_imu] : icm_imus) {
        Comms::Sendable<ICMImuData> icm_imu_sendable;
        icm_imu_sendable.data = icm_imu.get_data_for_comms();
        icm_imu_sendable.send_to_comms();
    }

    for (auto& [lsm_imu_name, lsm_imu] : lsm_imus) {
        Comms::Sendable<LSMImuData> lsm_imu_sendable;
        lsm_imu_sendable.data = lsm_imu.get_data_for_comms();
        lsm_imu_sendable.send_to_comms();
    }

    for (auto& [d200_lidar_name, d200_lidar] : d200_lidars) {
        Comms::Sendable<D200LidarData> d200_lidar_sendable;
        d200_lidar_sendable.data = d200_lidar.get_data_for_comms();
        d200_lidar_sendable.send_to_comms();
    }

    for (auto& [limit_switch_name, limit_switch] : limit_switches) {
        Comms::Sendable<LimitSwitchData> limit_switch_sendable;
        limit_switch_sendable.data = limit_switch.get_data_for_comms();
        limit_switch_sendable.send_to_comms();
    }

    for (auto& [acs712_current_sensor_name, acs712_current_sensor] : acs712_current_sensors) {
        Comms::Sendable<CurrentSensorData> current_sensor_sendable;
        current_sensor_sendable.data = acs712_current_sensor.get_data_for_comms();
        current_sensor_sendable.send_to_comms();
    }

    for (auto& [tof_sensor_name, tof_sensor] : tof_sensors) {
        Comms::Sendable<TOFSensorData> tof_sensor_sendable;
        tof_sensor_sendable.data = tof_sensor.get_data_for_comms();
        tof_sensor_sendable.send_to_comms();
    }

    for (auto& [stereo_cam_trigger_name, stereo_cam_trigger] : stereo_cam_triggers) {
        Comms::Sendable<StereoCamTriggerData> stereo_cam_trigger_sendable;
        stereo_cam_trigger_sendable.data = stereo_cam_trigger.get_data_for_comms();
        stereo_cam_trigger_sendable.send_to_comms();
    }
}

std::optional<BuffEncoder*> SensorManager::get_buff_encoder_by_name(NewConfig::BuffEncoderName name) {
    for(auto& [buff_enc_name, buff_enc] : buff_encoders) {
        if(buff_enc_name == name) {
            return &buff_enc;
        }
    }
    return std::nullopt;
}

std::optional<RevEncoder*> SensorManager::get_rev_encoder_by_name(NewConfig::RevEncoderName name) {
    for(auto& [rev_enc_name, rev_enc] : rev_encoders) {
        if(rev_enc_name == name) {
            return &rev_enc;
        }
    }
    return std::nullopt;
}

std::optional<ICM20649*> SensorManager::get_icm_sensor_by_name(NewConfig::ImuName name) {
    for (auto& [icm_imu_name, icm_imu] : icm_imus) {
        if (icm_imu_name == name) {
            return &icm_imu;
        }
    }
    return std::nullopt;
}

std::optional<LSM6DSOX*> SensorManager::get_lsm_sensor_by_name(NewConfig::ImuName name) {
    for (auto& [lsm_imu_name, lsm_imu] : lsm_imus) {
        if (lsm_imu_name == name) {
            return &lsm_imu;
        }
    }
    return std::nullopt;
}

std::optional<D200LD14P*> SensorManager::get_lidar_sensor_by_name(NewConfig::D200LidarName name) {
    for (auto& [d200_lidar_name, d200_lidar] : d200_lidars) {
        if (d200_lidar_name == name) {
            return &d200_lidar;
        }
    }
    return std::nullopt;
}

std::optional<LimitSwitch*> SensorManager::get_limit_switch_by_name(NewConfig::LimitSwitchName name) {
    for(auto& [limit_switch_name, limit_switch] : limit_switches) {
        if(limit_switch_name == name) {
            return &limit_switch;
        }
    }
    return std::nullopt;
}

std::optional<ACS712*> SensorManager::get_current_sensor_by_name(NewConfig::CurrentSensorName name) {
    for(auto& [current_sensor_name, current_sensor] : acs712_current_sensors) {
        if(current_sensor_name == name) {
            return &current_sensor;
        }
    }
    return std::nullopt;
}

std::optional<TOFSensor*> SensorManager::get_tof_sensor_by_name(NewConfig::TOFSensorName name) {
    for(auto& [tof_sensor_name, tof_sensor] : tof_sensors) {
        if(tof_sensor_name == name) {
            return &tof_sensor;
        }
    }
    return std::nullopt;
}

std::optional<StereoCamTrigger*> SensorManager::get_stereo_cam_trigger_by_name(NewConfig::StereoCameraTriggerName name) {
    for (auto& [stereo_cam_trigger_name, stereo_cam_trigger] : stereo_cam_triggers) {
        if (stereo_cam_trigger_name == name) {
            return &stereo_cam_trigger;
        }
    }
    return std::nullopt;
}