#pragma once

#include <stdint.h>	    // uintX_t

/// @brief Comms packet flags
/// @note These flags are used to indicate the state of the packet
enum class PacketFlags : uint8_t {
    NONE        = 0,
    CONFIGURED  = 1 << 0,    // 0x01
};

/// @brief The start of frame (SOF) byte
constexpr uint8_t PACKET_SOF = 0xAB;

/// @brief Comms packet header struct
struct PacketHeader {
    /// @brief Starter byte (Start of Frame)
    uint8_t SOF = PACKET_SOF; // must be first
    /// @brief Info flags for this packet
    PacketFlags flags = PacketFlags::NONE;
    /// @brief A sequential ID of this packet. Value is incremented every time a packet is sent
    uint16_t sequence = 0;
};

/// @brief Size of a packet header
constexpr uint32_t PACKET_HEADER_SIZE = (sizeof(PacketHeader));

/// @brief The largest alignment block of the packet header
constexpr uint32_t PACKET_HEADER_ALIGNMENT = (alignof(PacketHeader));