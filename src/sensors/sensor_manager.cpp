#include "sensor_manager.hpp"

#include "sensors/buff_encoder.hpp"
#include "sensors/rev_encoder.hpp"

#include "sensors/ICM20649.hpp"
#include "sensors/LSM6DSOX.hpp"

#include "sensors/d200.hpp"
#include "sensors/limit_switch.hpp"
#include "sensors/StereoCamTrigger.hpp"

SensorManager::SensorManager() {}

SensorManager::~SensorManager() {
    Serial.println("Ending SPI");
    SPI.end();
	SPI1.end();
    Serial.println("SPI Ended");
}

void SensorManager::init(const Cfg::RobotConfig& config_data) {
    // start SPI
    Serial.println("Starting SPI");
    SPI.begin();
	SPI1.begin();
    Serial.println("SPI Started");

    configure_sensors(config_data);

    initialize_sensors();
}

void SensorManager::configure_sensors(const Cfg::RobotConfig& config_data) {
    for (const auto& buff_encoder_config : config_data.buff_encoders) {
        sensors.emplace(buff_encoder_config.encoder_name, std::make_shared<BuffEncoder>(buff_encoder_config));
    }

    for (const auto& rev_encoder_config : config_data.rev_encoders) {
        sensors.emplace(rev_encoder_config.encoder_name, std::make_shared<RevEncoder>(rev_encoder_config));
    }

    for (const auto& icm_config : config_data.icm_imus) {
        Serial.printf("Configuring ICM20649 with name %u\n", static_cast<uint32_t>(icm_config.imu_name));
        sensors.emplace(icm_config.imu_name, std::make_shared<ICM20649>(icm_config));
    }

    for (const auto& lsm_config : config_data.lsm_imus) {
        sensors.emplace(lsm_config.imu_name, std::make_shared<LSM6DSOX>(lsm_config));
    }

    for (const auto& d200_lidar_config : config_data.d200_lidars) {
        sensors.emplace(d200_lidar_config.lidar_name, std::make_shared<D200LD14P>(d200_lidar_config));
    }

    for (const auto& limit_switch_config : config_data.limit_switches) {
        sensors.emplace(limit_switch_config.switch_name, std::make_shared<LimitSwitch>(limit_switch_config));
    }

    for (const auto& stereo_cam_trigger_config : config_data.stereo_cam_triggers) {
        sensors.emplace(stereo_cam_trigger_config.camera_trigger_name, std::make_shared<StereoCamTrigger>(stereo_cam_trigger_config));
    }
}

void SensorManager::initialize_sensors(){
    for(auto& [sensor_name, sensor] : sensors) {
        sensor->init();
    }
}
void SensorManager::request_read() {
	    for(auto& [sensor_name, sensor] : sensors) {
        sensor->request_read();
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
void SensorManager::print_sensors_live() {
    for(auto& [sensor_name, sensor] : sensors) {
        sensor->print_live_data(); 
    }
}
