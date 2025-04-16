#include "firmware_data.hpp"

#if defined(HIVE)
#include "modules/comms/comms_layer.hpp"    // for CommsLayer
#include "modules/hive/environment.hpp"     // for Hive::env
#include <stdexcept>                        // for std::runtime_error
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
    case TypeLabel::EstimatedState: {
        // place the data in the mega struct
        estimated_state = *static_cast<EstimatedState*>(data);

    #if defined(HIVE)
        // update the state lookup history
        Hive::env->comms_layer->update_state_lookback(&estimated_state);
    #endif

        break;
    }
    case TypeLabel::TransmitterData: {
        // place the data in the mega struct
        transmitter_data = *static_cast<TransmitterData*>(data);
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
        // TODO: implement this on firmware
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
    
