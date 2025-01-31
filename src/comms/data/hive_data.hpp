#pragma once

#include "config_data.hpp"
#include "robot_state.hpp"

namespace Comms {

struct HiveData{
    ConfigData config_data;
    RobotState target_state;
    RobotState override_state;
};

} // namespace Comms