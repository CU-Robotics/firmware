#include "hive_data.hpp"
#include "comms_data.hpp"
#include "config_data/sensor.hpp"

#if defined(HIVE)
#include "modules/comms/comms_layer.hpp"    // for CommsLayer
#include "modules/hive/environment.hpp"     // for Hive::env
#include <doctest/doctest.h>                // for doctest
#elif defined(FIRMWARE)
#include "comms/comms_layer.hpp"            // for CommsLayer
#endif

namespace Comms {

void HiveData::set_data(CommsData* data) {
    Serial.printf("HiveData::set_data received type label %d\n", static_cast<uint8_t>(data->type_label));
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
    case TypeLabel::ControllerConfig: {
        Cfg::Controller* controller = static_cast<Cfg::Controller*>(data);
        config.controllers.push_back(*controller);
        config.num_sections_received++;
        break;
    }
    case TypeLabel::MotorConfig: {
        Cfg::Motor* motor = static_cast<Cfg::Motor*>(data);
        config.motors.push_back(*motor);
        config.num_sections_received++;
        break;
    }
    case TypeLabel::StateConfig: {
        Cfg::State* state_info = static_cast<Cfg::State*>(data);
        config.states.push_back(*state_info);
        config.num_sections_received++;
        break;
    }
    case TypeLabel::BuffEncoderConfig: {
        Cfg::BuffEncoder* buff_encoder = static_cast<Cfg::BuffEncoder*>(data);
        config.buff_encoders.push_back(*buff_encoder);
        config.num_sections_received++;
        break;
    }
    case TypeLabel::IcmImuConfig: {
        Cfg::IcmImu* icm_imu = static_cast<Cfg::IcmImu*>(data);
        config.icm_imus.push_back(*icm_imu);
        config.num_sections_received++;
        break;
    }
    case TypeLabel::D200LidarConfig: {
        Cfg::D200Lidar* d200_lidar = static_cast<Cfg::D200Lidar*>(data);
        config.d200_lidars.push_back(*d200_lidar);
        config.num_sections_received++;
        break;
    }
    case TypeLabel::EstimatorConfig: {
        Cfg::Estimator* estimator = static_cast<Cfg::Estimator*>(data);
        config.estimators.push_back(*estimator);
        config.num_sections_received++;
        break;
    }
    case TypeLabel::StereoCameraTriggerConfig: {
        Cfg::StereoCamTrigger* stereo_camera_trigger = static_cast<Cfg::StereoCamTrigger*>(data);
        config.stereo_cam_triggers.push_back(*stereo_camera_trigger);
        config.num_sections_received++;
        break;
    }
    case TypeLabel::ConfigStart: {
        Cfg::ConfigStart* config_start = static_cast<Cfg::ConfigStart*>(data);
        config.config_start = *config_start;
        break;
    }
    default:
    #if defined(HIVE)
        throw std::runtime_error("Invalid type label given to place in mega struct");
    #elif defined(FIRMWARE)
        // assert(false && "Invalid type label given to place in mega struct");
        Serial.printf("Invalid type label given to place in mega struct: %d\n", static_cast<uint8_t>(data->type_label));
    #endif
    }
}

}   // namespace Comms

#if defined(HIVE)

TEST_CASE("setting hive data structs") {
    Comms::HiveData hive_data;

    TestData test_data;
    test_data.x = 55;
    hive_data.set_data(&test_data);
    CHECK(hive_data.test_data.x == 55);

    BigTestData big_test_data;
    big_test_data.blah[1] = 55;
    hive_data.set_data(&big_test_data);
    CHECK(hive_data.big_test_data.blah[1] == 55);

    TargetState target_state;
    target_state.time = 55;
    hive_data.set_data(&target_state);
    CHECK(hive_data.target_state.time == 55);

    OverrideState override_state;
    override_state.time = 55;
    hive_data.set_data(&override_state);
    CHECK(hive_data.override_state.time == 55);

    ConfigSection config_section;
    config_section.section_id = 55;
    hive_data.set_data(&config_section);
    CHECK(hive_data.config_section.section_id == 55);
}

#endif  // defined (HIVE)