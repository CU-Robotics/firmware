#pragma once

#include "packet_header.hpp"

namespace Comms {

/// @brief The maximum size of a HID packet, including the header
constexpr uint32_t HID_PACKET_MAX_SIZE = (1023u);

/// @brief The size of the payload-usable space in a HID packet
constexpr uint32_t HID_PACKET_PAYLOAD_SIZE = (HID_PACKET_MAX_SIZE - PACKET_HEADER_SIZE);

/// @brief A complete HID packet, including the header and payload
struct HIDPacket {
    /// @brief The header of the packet
    PacketHeader header = {};
    /// @brief The payload of the packet
    uint8_t data[HID_PACKET_PAYLOAD_SIZE] = { 0 };

    /// @brief The first byte of the packet itself
    uint8_t* data_start() { return reinterpret_cast<uint8_t*>(this); }

    /// @brief The first byte of the payload
    uint8_t* payload() { return data; }
};

/// @brief The largest alignment block of the HID packet
constexpr uint32_t HID_PACKET_ALIGNMENT = (alignof(HIDPacket));

}   // end Comms namespace