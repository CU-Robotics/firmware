#pragma once

#include <stdint.h>	// uintX_t
#include <string.h>	// memset

namespace Comms {

/// @brief Comms packet header struct
struct EthernetPacketHeader {
	/// @brief Time stamp of when this packet was sent
	uint64_t time_stamp = 0;
	/// @brief A sequential ID of this packet. Value is incremented every time a packet is sent
	uint32_t sequence = 0;
	/// @brief Size of the attached packet payload
	uint16_t payload_size = 0;
	/// @brief The type of this packet
	uint8_t type = 0;
	/// @brief The flags of this packet
	uint8_t flags = 0;
};

/// @brief Size of a packet header
constexpr uint32_t ETHERNET_PACKET_HEADER_SIZE = (sizeof(Comms::EthernetPacketHeader));

/// @brief The max packet size
constexpr uint32_t ETHERNET_PACKET_MAX_SIZE = (4096u);

/// @brief The max packet payload size
constexpr uint32_t ETHERNET_PACKET_PAYLOAD_MAX_SIZE = (Comms::ETHERNET_PACKET_MAX_SIZE - Comms::ETHERNET_PACKET_HEADER_SIZE);

/// @brief Comms packet payload struct
struct EthernetPacketPayload{
	/// @brief The raw data array in bytes
	uint8_t data[Comms::ETHERNET_PACKET_PAYLOAD_MAX_SIZE] = { 0 };
};

/// @brief A complete comms packet
struct EthernetPacket {
	/// @brief Packet header
	Comms::EthernetPacketHeader header{};
	/// @brief Packet payload
	Comms::EthernetPacketPayload payload{};

	/// @brief Returns the starting address of this packet. Used in data processing
	/// @return The first byte of the packet, specifically the address to the header
	uint8_t* data() {
		return reinterpret_cast<uint8_t*>(&header);
	}

	/// @brief Clears the packet to all 0s
	void clear() {
		memset(&header.time_stamp, 0, sizeof(Comms::EthernetPacketHeader));
		memset(&payload.data, 0, sizeof(Comms::EthernetPacketPayload));
	}
};

/// @brief The possible packet types, these specify special behavior
enum EthernetPacketType {
	DATA = 0,
	PRIORITY = 1,
	HANDSHAKE = 2,
	DEBUG = 3,
	EOT = 4,
};

/// @brief The possible packet flags, these specify the general contents
enum EthernetPacketFlags {
	NORMAL = 0,
	CONFIG = 1,
	STATE_OVERRIDE = 2,
	ACK = 3,
};

}	// namespace Comms