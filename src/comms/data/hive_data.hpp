#pragma once

#if defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#include "modules/comms/data/data_structs.hpp"  // for shared data structs
#elif defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/data/data_structs.hpp"          // for shared data structs
#include "comms/config_data/robot_config.hpp"   // for RobotConfig
#endif

namespace Comms {

/// @brief Megastruct for receiving data from Hive, filled on firmware.
struct HiveData {
    /// @brief Set a data section in the mega struct.
    /// @param data The data to be set.
    void set_data(CommsData* data);
    
    /// @brief Test data
    TestData test_data;
    /// @brief Big test data
    BigTestData big_test_data;

    /// @brief Target state received from Hive; This is used as a reference for firmware to follow using its reference governor, state estimator and controllers.
    TargetState target_state_data;

    /// @brief Override state received from Hive; This is used to override the robot state estimate on firmware with a new one.
    OverrideState override_state_data;

    /// @brief The configuration data filled as config sections are received over comms. This should only be used after all config sections have been received.
    Cfg::RobotConfig config;
};

} // namespace Comms
