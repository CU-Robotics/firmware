#include "comms/hid_comms.hpp"

namespace Comms {
    HIDComms::HIDComms() {}
    bool HIDComms::is_initialized() const { return false; }
    bool HIDComms::is_connected() const { return false; }
    bool HIDComms::recv_packet(HIDPacket&) { return false; }
    bool HIDComms::send_packet(HIDPacket&) { return false; }
}