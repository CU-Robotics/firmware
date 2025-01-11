#pragma once

#include <doctest/doctest.h>
#include <vector>
#include <queue>

#include "comms_data.hpp"
#include "modules/hive/robot_state.hpp"


namespace Comms {

struct FirmwareData : public CommsData {
    FirmwareData() {
        data_header = DataType::FirmwareInfo;   // always want firmware to specify its data header by default
    }
    struct SensorsData {
        struct LidarStuff;
        struct IMUStuff;
    } sensors_data;
    Hive::RobotState estimated_state;
};

} // namespace Comms