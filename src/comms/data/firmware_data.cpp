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
    case TypeLabel::CommsRefData: {
        CommsRefData ref_data = *static_cast<CommsRefData*>(data);
        this->ref_data = ref_data;
        break;
    }
    case TypeLabel::ConfigurationStatus: {
        ConfigurationStatusData config_status_data = *static_cast<ConfigurationStatusData*>(data);
        this->config_status_data = config_status_data;
        break;
    }
    default:
        safety::safety_procedure("FirmwareData::set_data: Invalid type label given to place in mega struct: %u", static_cast<uint16_t>(data->type_label));
    }

}

}   // namespace Comms