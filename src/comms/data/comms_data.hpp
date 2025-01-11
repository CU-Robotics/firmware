#pragma once

#include <cstdint>

namespace Comms {

/// @brief base class for all data structs that want to be sent over comms.
class CommsData {
public:
    enum class DataType : uint8_t {
        NONE = 0x00,
        Config = 0x01,
        TargetState = 0x02,
        OverrideState = 0x03,
        // RandomVariables = 0x04,  // is this needed for anything? @ kyle 3
        FirmwareInfo = 0x04,
    } data_header = DataType::NONE;

    enum class CriticalMode : uint8_t {
        Critical,
        NonCritical
    } critical_mode;
};

} // namespace Comms