#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#if defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#elif defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#endif

#include <stdint.h>     // uintN_t

#include "test_data.hpp"

#include "buff_encoder_data.hpp"

#include "rev_encoder_data.hpp"

#include "icm_sensor_data.hpp"

#include "lidar_data_packet_si.hpp"
#include "dr16_data.hpp"
#include "ET16S_data.hpp"

#include "robot_state_data.hpp"

#include "comms_ref_data.hpp"

#include "stereo_cam_trigger_data.hpp"

#include "ref_drawing_data.hpp"

#endif // DATA_STRUCTS_HPP
