#pragma once

#include "packet_header.hpp"    // for PacketHeader
#include <string.h>             // for memset

namespace Comms {

/// @brief The max packet size
/// @note The max size this value can be currently is 7392 bytes. This was found through testing and is roughly the size of 5 IP fragments.
///       Anything past this value fails to be received fully by Linux.
///       I belive the issue is that Teensy is failing to fragment the packet properly. I've only ever seen a max of 5 fragments being sent by Teensy
///       even though packets could comprise more fragments. I am unsure on where in the QNEthernet stack this is occuring and do not have time to investigate.
///       6KB is a safe value to use for now, as 7KB is a bit too close for comfort.
///       A possible work around to this is to run our own fragmentation layer on top of the QNEthernet stack, but this is a lot of work.
constexpr uint32_t ETHERNET_PACKET_MAX_SIZE = (6u * 1024u); // 6 KB = 6144 bytes

/// @brief The max packet payload size
constexpr uint32_t ETHERNET_PACKET_PAYLOAD_SIZE = (ETHERNET_PACKET_MAX_SIZE - PACKET_HEADER_SIZE);

/// @brief A complete ethernet packet, including the header and payload
struct EthernetPacket {
    /// @brief The header of the packet
    PacketHeader header = {};
    /// @brief The payload of the packet
    uint8_t data[ETHERNET_PACKET_PAYLOAD_SIZE] = { 0 };

    /// @brief The first byte of the packet itself
    /// @return A pointer to the first byte of the packet
    uint8_t* data_start() { return reinterpret_cast<uint8_t*>(this); }

    /// @brief The first byte of the payload
    /// @return A pointer to the first byte of the payload
    uint8_t* payload() { return data; }

    /// @brief Clear the packet
    /// @note This clears the packet to all 0s
    void clear() {
        header.SOF = PACKET_SOF;
        header.flags = PacketFlags::NONE;
        header.sequence = 0;
        memset(data, 0, ETHERNET_PACKET_PAYLOAD_SIZE);
    }
};

/// @brief The largest alignment block of the ethernet packet
constexpr uint32_t ETHERNET_PACKET_ALIGNMENT = (alignof(EthernetPacket));

}	// namespace Comms