#pragma once 
#include "comms/config_data/hardware_serial_port.hpp"
#include "comms/data/comms_data.hpp"

namespace Cfg {
/// @brief Different types of transmitters that can be used on the robot. 
enum class TransmitterType : uint32_t{
    DR16,
    ET16S
};

/// @brief Configuration struct for the DR16 transmitter.
struct DR16 {

};

/// @brief Configuration struct for the ET16S transmitter.
struct ET16S {

};
/// @brief The `Transmitter` struct represents the configuration for a transmitter, including its type and any specific configuration data for that type.
struct Transmitter : Comms::CommsData {
    /// @brief The type of transmitter that is being configured. This determines which specific configuration struct is used for this transmitter.
    TransmitterType transmitter_type = TransmitterType::DR16;
    /// @brief Configuration data for the DR16 transmitter. This is only used if the transmitter_type is DR16.
    DR16 dr16;
    /// @brief Configuration data for the ET16S transmitter. This is only used if the transmitter_type is ET16S.
    ET16S et16s;
};

} // namespace Cfg