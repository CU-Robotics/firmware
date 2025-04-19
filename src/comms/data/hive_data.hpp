#pragma once

#if defined(HIVE)
#include "modules/comms/data/comms_data.hpp"    // for CommsData
#include "modules/comms/data/data_structs.hpp"  // for shared data structs
#elif defined(FIRMWARE)
#include "comms/data/comms_data.hpp"            // for CommsData
#include "comms/data/data_structs.hpp"          // for shared data structs
#endif

namespace Comms {

/// @brief Megastruct for receiving data from Hive, filled on firmware.
struct HiveData {
    /// @brief Set a data section in the mega struct.
    /// @param data The data to be set.
    /// @warning This is not thread safe, call this on local copies only
    void set_data(CommsData* data);
    
    /// @brief Test data
    TestData test_data;
    /// @brief Big test data
    BigTestData big_test_data;

    /// @brief Target state
    TargetState target_state;

    /// @brief Override state
    OverrideState override_state;

    /// @brief Config section
    ConfigSection config_section;
};

} // namespace Comms
