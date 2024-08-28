#pragma once 

#include <Arduino.h>

#include "ethernet_packet.hpp"

namespace Comms {

/// @brief A status struct holding diagnostic data about ethernet comms
struct EthernetStatus {
	/// @brief The number of packets sent
	uint64_t packets_sent = 0;
	/// @brief The number of packets that failed to send
	uint64_t packets_sent_failed = 0;
	/// @brief The number of packets received
	uint64_t packets_received = 0;
	/// @brief The number of packets that failed to receive
	uint64_t packets_received_failed = 0;
	/// @brief The timestamp of the last successful handshake
	uint32_t handshake_time = 0;
};

}	// namespace Comms