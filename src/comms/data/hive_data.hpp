#pragma once

#include "config_data.hpp"
#include "modules/hive/robot_state.hpp"

namespace Comms {

struct HiveData : public CommsData {
    ConfigData config_data;
    Hive::RobotState target_state;
    Hive::RobotState override_state;
};

} // namespace Comms