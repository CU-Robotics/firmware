#include "hive_data.hpp"
#include "comms_data.hpp"
#include "config_data/sensor.hpp"

#include "comms/comms_layer.hpp"            // for CommsLayer

extern "C" void reset_teensy(void);

namespace Comms {

void HiveData::set_data(CommsData* data) {
    // Serial.printf("HiveData::set_data received type label %d\n", static_cast<uint8_t>(data->type_label));
    // place the data in the mega struct
    switch (data->type_label) {
    case TypeLabel::TestData: {
        TestData *test = static_cast<TestData*>(data);
        // TODO: why does doing test_data = *test; not work?
        memcpy(&test_data, test, sizeof(TestData));
        break;
    }
    case TypeLabel::BigTestData: {
        BigTestData *big_test = static_cast<BigTestData*>(data);
        memcpy(&big_test_data, big_test, sizeof(BigTestData));
        // big_test_data = *static_cast<BigTestData*>(data);
        break;
    }
    case TypeLabel::TargetState: {
        TargetState* target = static_cast<TargetState*>(data);
        memcpy(&target_state_data, target, sizeof(TargetState));
        // target_state = *static_cast<TargetState*>(data);
        break;
    }
    case TypeLabel::OverrideState: {
        OverrideState* o_state = static_cast<OverrideState*>(data);
        memcpy(&override_state_data, o_state, sizeof(OverrideState));
        // override_state = *static_cast<OverrideState*>(data);
        break;
    }
    case TypeLabel::ConfigStart: {
        Cfg::ConfigStart* config_start = static_cast<Cfg::ConfigStart*>(data);
        if (config.is_configured()) {
            Serial.printf("Recieved config start packet with %d sections; rebooting teensy to reconfigure.\n", config.config_start.num_config_sections);
            delay(100);
            reset_teensy();
        }
        config.config_start = *config_start;

        break;
    }
    case TypeLabel::ControllerConfig: {
        Cfg::Controller* controller = static_cast<Cfg::Controller*>(data);
        config.controllers.push_back(*controller);
        config.num_sections_received++;
        Serial.printf("Controller %u received\n", static_cast<uint32_t>(controller->controller_type));
        break;
    }
    case TypeLabel::MotorConfig: {
        Cfg::Motor* motor = static_cast<Cfg::Motor*>(data);
        config.motors.push_back(*motor);
        config.num_sections_received++;
        Serial.printf("Motor %u received\n", static_cast<uint32_t>(motor->motor_name));
        break;
    }
    case TypeLabel::StateConfig: {
        Cfg::State* state_info = static_cast<Cfg::State*>(data);
        config.states.push_back(*state_info);
        config.num_sections_received++;
        Serial.printf("State %u received\n", static_cast<uint32_t>(state_info->name));
        break;
    }
    case TypeLabel::BuffEncoderConfig: {
        Cfg::BuffEncoder* buff_encoder = static_cast<Cfg::BuffEncoder*>(data);
        config.buff_encoders.push_back(*buff_encoder);
        config.num_sections_received++;
        Serial.printf("Buff encoder %u received\n", static_cast<uint32_t>(buff_encoder->encoder_name));
        break;
    }
    case TypeLabel::IcmImuConfig: {
        Cfg::IcmImu* icm_imu = static_cast<Cfg::IcmImu*>(data);
        config.icm_imus.push_back(*icm_imu);
        config.num_sections_received++;
        Serial.printf("ICM IMU %u received\n", static_cast<uint32_t>(icm_imu->imu_name));
        break;
    }
    case TypeLabel::D200LidarConfig: {
        Cfg::D200Lidar* d200_lidar = static_cast<Cfg::D200Lidar*>(data);
        config.d200_lidars.push_back(*d200_lidar);
        config.num_sections_received++;
        Serial.printf("D200 Lidar %u received\n", static_cast<uint32_t>(d200_lidar->lidar_name));
        break;
    }
    case TypeLabel::EstimatorConfig: {
        Cfg::Estimator* estimator = static_cast<Cfg::Estimator*>(data);
        config.estimators.push_back(*estimator);
        config.num_sections_received++;
        Serial.printf("Estimator %u received\n", static_cast<uint32_t>(estimator->estimator_type));
        break;
    }
    case TypeLabel::StereoCameraTriggerConfig: {
        Cfg::StereoCamTrigger* stereo_camera_trigger = static_cast<Cfg::StereoCamTrigger*>(data);
        config.stereo_cam_triggers.push_back(*stereo_camera_trigger);
        config.num_sections_received++;
        Serial.printf("Stereo camera trigger %u received\n", static_cast<uint32_t>(stereo_camera_trigger->camera_trigger_name));
        break;
    }
    case TypeLabel::TransmitterConfig: {
        Cfg::Transmitter* transmitter = static_cast<Cfg::Transmitter*>(data);
        config.transmitter = *transmitter;
        config.num_sections_received++;
        Serial.printf("Transmitter %u received\n", static_cast<uint32_t>(transmitter->transmitter_type));
        break;
    }
    case TypeLabel::StartStereoTrigger: {
        StartStereoTrigger* start_trigger = static_cast<StartStereoTrigger*>(data);
        stereo_cam_start_stop.start_received = true;
        Serial.printf("Start stereo trigger for %u received\n", static_cast<uint32_t>(start_trigger->camera_trigger_name));
        break;
    }
    case TypeLabel::StopStereoTrigger: {
        StopStereoTrigger* stop_trigger = static_cast<StopStereoTrigger*>(data);
        stereo_cam_start_stop.stop_received = true;
        Serial.printf("Stop stereo trigger for %u received\n", static_cast<uint32_t>(stop_trigger->camera_trigger_name));
        break;
    }
    default:
        safety::safety_procedure("HiveData::set_data: Invalid type label given to place in mega struct: %u\n", static_cast<uint32_t>(data->type_label));
    }
}

}   // namespace Comms