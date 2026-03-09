#pragma once 
#include "comms/config_data/hardware_serial_port.hpp"
#include "comms/data/comms_data.hpp"

namespace Cfg {

enum class TransmitterType : uint32_t{
    DR16,
    ET16S
};

struct DR16 {

};

struct ET16S {

};

struct Transmitter : Comms::CommsData {
    TransmitterType transmitter_type;
    DR16 dr16;
    ET16S et16s;
};

} // namespace Cfg