#include "firmware_data.hpp"
#include "icm_sensor_data.hpp"
#include "utils/safety.hpp"

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
    case TypeLabel::TargetState: {
        // place the data in the mega struct
        temp_reference = *static_cast<TargetState*>(data);
        break;
    }
    case TypeLabel::EstimatedState: {
        // place the data in the mega struct
        estimated_state = *static_cast<EstimatedState*>(data);
        break;
    }
    case TypeLabel::DR16Data: {
        // place the data in the mega struct
        dr16_data = *static_cast<DR16Data*>(data);
        break;
    }
    case TypeLabel::ET16SData: {
        // place the data in the mega struct
        et16s_data = *static_cast<ET16SData*>(data);
        break;
    }
    case TypeLabel::BuffEncoderData: {
        //determine if the data is for yaw or pitch
        BuffEncoderData single_buff_encoder_data = *static_cast<BuffEncoderData*>(data);
        buff_encoder_data_map.insert_or_assign(single_buff_encoder_data.encoder_name, single_buff_encoder_data);
        printf("buff encoder angle: %f\n", single_buff_encoder_data.m_angle);
        break;
    }
    case TypeLabel::ICMSensorData: {
        // place the data in the mega struct
        ICMSensorData icm_sensor = *static_cast<ICMSensorData*>(data);
        icm_sensor_data_map.insert_or_assign(icm_sensor.imu_name, icm_sensor);
        break;
    }
    case TypeLabel::LsmSensorData: {
        // place the data in the mega struct
        LsmSensorData lsm_sensor = *static_cast<LsmSensorData*>(data);
        lsm_sensor_data_map.insert_or_assign(lsm_sensor.imu_name, lsm_sensor);
        break;
    }
    case TypeLabel::LimitSwitchData: {
        // place the data in the mega struct
        LimitSwitchData limit_switch_data = *static_cast<LimitSwitchData*>(data);
        limit_switch_data_map.insert_or_assign(limit_switch_data.switch_name, limit_switch_data);
        break;
    }
    case TypeLabel::RevEncoderData: {
        // place the data in the mega struct
        RevSensorData rev_sensor_data = *static_cast<RevSensorData*>(data);
        rev_sensor_data_map.insert_or_assign(rev_sensor_data.encoder_name, rev_sensor_data);
        break;
    }
    case TypeLabel::StereoCamTriggerData: {
        // place the data in the mega struct
        StereoCamTriggerData stereo_cam_trigger_data = *static_cast<StereoCamTriggerData*>(data);
        stereo_camera_trigger_data_map.insert_or_assign(stereo_cam_trigger_data.camera_trigger_name, stereo_cam_trigger_data);
        break;
    }
    case TypeLabel::MotorStateData: {
        // place the data in the mega struct
        MotorStateData motor_state = *static_cast<MotorStateData*>(data);
        motor_state_data_map.insert_or_assign(motor_state.motor_name, motor_state);
        break;
    }
    case TypeLabel::LidarDataPacketSI: {
        //determine which lidar sensor the data is for
        LidarDataPacketSI single_lidar_data = *static_cast<LidarDataPacketSI*>(data);
        lidar_data_map.insert_or_assign(single_lidar_data.lidar_name, single_lidar_data);
        break;
    }
    case TypeLabel::ControllerOutputData: {
        // place the data in the mega struct
        controller_output_data = *static_cast<ControllerOutputData*>(data);
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
        safety::safety_procedure("Invalid type label given to place in mega struct: %u", static_cast<uint16_t>(data->type_label));
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

    EstimatedState estimated_state;
    estimated_state.time = 55;
    firmware_data.set_data(&estimated_state);
    CHECK(firmware_data.estimated_state.time == 55);

    BuffEncoderData buff_encoder_data;
    buff_encoder_data.m_angle = 55;
    buff_encoder_data.id = 0;
    firmware_data.set_data(&buff_encoder_data);
    CHECK(firmware_data.buff_encoder_data_map.at(Cfg::SensorName::Yaw).m_angle == 55);
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

    ET16SData et16s_data;
    et16s_data.mouse_x = 55;
    firmware_data.set_data(&et16s_data);
    CHECK(firmware_data.et16s_data.mouse_x == 55);

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
