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

#include "rev_sensor_data.hpp"

#include "icm_sensor_data.hpp"

#include "tof_sensor_data.hpp"

#include "lidar_data_packet_si.hpp"

#include "transmitter_data.hpp"

#include "robot_state_data.hpp"

#include "config_section.hpp"

#include "comms_ref_data.hpp"

#endif // DATA_STRUCTS_HPP
