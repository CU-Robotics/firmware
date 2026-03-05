#include "sensor_manager.hpp"

#include "sensors/ICM20649.hpp"
#include "sensors/buff_encoder.hpp"


SensorManager::SensorManager() {}

SensorManager::~SensorManager() {
    Serial.println("Ending SPI");
    SPI.end();
    Serial.println("SPI Ended");
}

void SensorManager::init(const Cfg::RobotConfig& config_data) {
    // start SPI
    Serial.println("Starting SPI");
    SPI.begin();
    Serial.println("SPI Started");

    configure_sensors(config_data);

    initialize_sensors();
}

void SensorManager::configure_sensors(const Cfg::RobotConfig& config_data) {
    for (const auto& buff_encoder_config : config_data.buff_encoders) {
        sensors.insert({buff_encoder_config.name, std::make_shared<BuffEncoder>(buff_encoder_config)});
    }

    for (const auto& rev_encoder_config : config_data.rev_encoders) {
        sensors.insert({rev_encoder_config.name, std::make_shared<RevEncoder>(rev_encoder_config)});
    }

    for (const auto& icm_config : config_data.icms) {
        sensors.insert({icm_config.name, std::make_shared<ICM20649>(icm_config)});
    }

    for (const auto& lsm_config : config_data.lsm_imus) {
        sensors.insert({lsm_config.name, std::make_shared<LSM6DSOX>(lsm_config)});
    }

    for (const auto& d200_lidar_config : config_data.d200_lidars) {
        sensors.insert({d200_lidar_config.name, std::make_shared<D200LD14P>(d200_lidar_config)});
    }

    for (const auto& limit_switch_config : config_data.limit_switches) {
        sensors.insert({limit_switch_config.name, std::make_shared<LimitSwitch>(limit_switch_config)});
    }

    for (const auto& stereo_cam_trigger_config : config_data.stereo_cam_triggers) {
        sensors.insert({stereo_cam_trigger_config.name, std::make_shared<StereoCamTrigger>(stereo_cam_trigger_config)});
    }
}

void SensorManager::initialize_sensors(){
    for(auto& [sensor_name, sensor] : sensors) {
        sensor->init();
    }
}

void SensorManager::read() {
    for(auto& [sensor_name, sensor] : sensors) {
        sensor->read();
    }
}

void SensorManager::send_to_comms() {
    for(auto& [sensor_name, sensor] : sensors) {
        sensor->send_to_comms();
    }
}