#include "firmware_data.hpp"

#if defined(HIVE)
#include "modules/comms/comms_layer.hpp"    // for CommsLayer
#include "modules/hive/environment.hpp"     // for Hive::env
#include <stdexcept>                        // for std::runtime_error
#include <doctest/doctest.h>                // for doctest
#elif defined(FIRMWARE)
#include "comms/comms_layer.hpp"            // for CommsLayer
#include <cassert>                          // for assert
#endif

namespace Comms {

void FirmwareData::set_data(CommsData* data) {
    switch (data->type_label) {
    case TypeLabel::TestData: {
        // place the data in the mega struct
        test_data = *static_cast<TestData*>(data);
        break;
    }
    case TypeLabel::BigTestData: {
        // place the data in the mega struct
        big_test_data = *static_cast<BigTestData*>(data);
        break;
    }
    case TypeLabel::LoggingData: {
        // place the data in the mega struct
        logging_data = *static_cast<LoggingData*>(data);
        break;
    }
    case TypeLabel::TempRobotState: {
        // place the data in the mega struct
        temp_robot_state = *static_cast<TempRobotState*>(data);
        break;
    }
    case TypeLabel::TargetState: {
        // place the data in the mega struct
        temp_reference = *static_cast<TargetState*>(data);
        break;
    }
    case TypeLabel::EstimatedState: {
        // place the data in the mega struct
        estimated_state = *static_cast<EstimatedState*>(data);

    #if defined(HIVE)
        // update the state lookup history
        Hive::env->comms_layer->add_estimated_state_to_lookup(&estimated_state);
    #endif

        break;
    }
    case TypeLabel::DR16Data: {
        // place the data in the mega struct
        dr16_data = *static_cast<DR16Data*>(data);
        break;
    }
    case TypeLabel::BuffEncoderData: {
        //determine if the data is for yaw or pitch
        BuffEncoderData* buff_encoder_data = static_cast<BuffEncoderData*>(data);
        if (buff_encoder_data->id == 0) {
            yaw_buff_encoder = *buff_encoder_data;
        } else if (buff_encoder_data->id == 1) {
            pitch_buff_encoder = *buff_encoder_data;
        }
        break;
    }
    case TypeLabel::RevEncoderData: {
        //determine which rev encoder the data is for
        RevSensorData* rev_encoder_data = static_cast<RevSensorData*>(data);
        if (rev_encoder_data->id == 0) {
            rev_sensor_0 = *rev_encoder_data;
        } else if (rev_encoder_data->id == 1) {
            rev_sensor_1 = *rev_encoder_data;
        } else if (rev_encoder_data->id == 2) {
            rev_sensor_2 = *rev_encoder_data;
        }
        break;
    }
    case TypeLabel::ICMSensorData: {
        // place the data in the mega struct
        icm_sensor = *static_cast<ICMSensorData*>(data);
        break;
    }
    case TypeLabel::TOFSensorData: {
        // place the data in the mega struct
        tof_sensor = *static_cast<TOFSensorData*>(data);
        break;
    }
    case TypeLabel::LidarDataPacketSI: {
        //determine which lidar sensor the data is for
    #if defined(HIVE)
        // TODO: implement this on firmware, do we actually need it on firmware?
        LidarDataPacketSI* lidar_sensor_data = static_cast<LidarDataPacketSI*>(data);
        if (lidar_sensor_data->id == 0) {
            lidars[0].push_back(*lidar_sensor_data);
        } else if (lidar_sensor_data->id == 1) {
            lidars[1].push_back(*lidar_sensor_data);
        }
    #endif
        break;
    }
    case TypeLabel::ConfigSection: {
        // place the data in the mega struct
        config_section = *static_cast<ConfigSection*>(data);
        break;
    }
    case TypeLabel::CommsRefData: {
        // place the data in the mega struct
    #if defined(HIVE)
        // TODO: implement this on firmware, do we actually need it on firmware?
        CommsRefData* comms_ref_data = static_cast<CommsRefData*>(data);
        ref_data.set_data(comms_ref_data->raw);
    #endif
        break;
    }
    default:
    #if defined(HIVE)
        throw std::runtime_error("Invalid type label given to place in mega struct: " + std::to_string(static_cast<uint8_t>(data->type_label)));
    #elif defined(FIRMWARE)
        assert(false && "Invalid type label given to place in mega struct");
    #endif
    }

}

}   // namespace Comms
    
#if defined(HIVE) 

TEST_CASE("setting firmware data structs") {
    Comms::FirmwareData firmware_data;
    
    TestData test_data;
    test_data.x = 55;
    firmware_data.set_data(&test_data);
    CHECK(firmware_data.test_data.x == 55);

    BigTestData big_test_data;
    big_test_data.blah[1] = 55;
    firmware_data.set_data(&big_test_data);
    CHECK(firmware_data.big_test_data.blah[1] == 55);

    TempRobotState temp_robot_state;
    temp_robot_state.time = 55;
    firmware_data.set_data(&temp_robot_state);
    CHECK(firmware_data.temp_robot_state.time == 55);

    EstimatedState estimated_state;
    estimated_state.time = 55;
    firmware_data.set_data(&estimated_state);
    CHECK(firmware_data.estimated_state.time == 55);

    BuffEncoderData buff_encoder_data;
    buff_encoder_data.m_angle = 55;
    buff_encoder_data.id = 0;
    firmware_data.set_data(&buff_encoder_data);
    CHECK(firmware_data.yaw_buff_encoder.m_angle == 55);
    buff_encoder_data.m_angle = 55;
    buff_encoder_data.id = 1;
    firmware_data.set_data(&buff_encoder_data);
    CHECK(firmware_data.yaw_buff_encoder.m_angle == 55);
    

    RevSensorData rev_sensor_data;
    rev_sensor_data.ticks = 55;
    rev_sensor_data.id = 0;
    firmware_data.set_data(&rev_sensor_data);
    CHECK(firmware_data.rev_sensor_0.ticks == 55);
    rev_sensor_data.ticks = 55;
    rev_sensor_data.id = 1;
    firmware_data.set_data(&rev_sensor_data);
    CHECK(firmware_data.rev_sensor_1.ticks == 55);
    rev_sensor_data.ticks = 55;
    rev_sensor_data.id = 2;
    firmware_data.set_data(&rev_sensor_data);
    CHECK(firmware_data.rev_sensor_2.ticks == 55);

    ICMSensorData icm_sensor_data;
    icm_sensor_data.accel_X = 55;
    firmware_data.set_data(&icm_sensor_data);
    CHECK(firmware_data.icm_sensor.accel_X == 55);

    TOFSensorData tof_sensor_data;
    tof_sensor_data.latest_distance = 55;
    firmware_data.set_data(&tof_sensor_data);
    CHECK(firmware_data.tof_sensor.latest_distance == 55);

    LidarDataPacketSI lidar_sensor_data;
    lidar_sensor_data.id = 0;
    lidar_sensor_data.lidar_speed = 55;
    firmware_data.set_data(&lidar_sensor_data);
    CHECK(firmware_data.lidars[0].back().lidar_speed == 55);
    lidar_sensor_data.id = 1;
    lidar_sensor_data.lidar_speed = 55;
    firmware_data.set_data(&lidar_sensor_data);
    CHECK(firmware_data.lidars[1].back().lidar_speed == 55);

    DR16Data dr16_data;
    dr16_data.mouse_x = 55;
    firmware_data.set_data(&dr16_data);
    CHECK(firmware_data.dr16_data.mouse_x == 55);

    ConfigSection config_section;
    config_section.section_id = 55;
    firmware_data.set_data(&config_section);
    CHECK(firmware_data.config_section.section_id == 55);

    CommsRefData comms_ref_data;
    comms_ref_data.raw[0] = 55;
    firmware_data.set_data(&comms_ref_data);
    CHECK(firmware_data.ref_data.game_status.raw[0] == 55);
}

#endif  // defined(HIVE)