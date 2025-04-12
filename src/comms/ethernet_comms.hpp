#pragma once

#include <Arduino.h>

#include <QNEthernet.h>
namespace qn = qindesign::network;

#include "utils/timing.hpp"				// for Timer
#include "comms/ethernet_packet.hpp"	// for EthernetPacket

// DEBUG define for displaying all comms errors/status updates
// This is very noisy on start up
// #define COMMS_DEBUG

namespace Comms {

/// @brief Ethernet Communications. This handles all comms between the Jetson and the Teensy via Ethernet
class EthernetComms {
public:
	/// @brief Defaulted constructor, does nothing
	EthernetComms() = default;
	/// @brief Defaulted constructor, does nothing
	~EthernetComms() = default;
	
public:
	/// @brief Initialize Ethernet comms and start the UDP connection
	/// @param data_rate (optional) The data rate in mbps
	/// @return True for success
	bool init(uint32_t data_rate = 95);

	/// @brief Send a packet to Hive
    /// @param packet The packet to send
    /// @return True if success
    bool send_packet(EthernetPacket& packet);

    /// @brief Receive a packet from Hive
    /// @param packet The packet to fill with data
    /// @return True if success
    bool recv_packet(EthernetPacket& packet);

	/// @brief Get the current connection status to Hive
    /// @return Connected or not
    bool is_connected() const;

    /// @brief Get the current initialized status of the Ethernet layer
    /// @return True if the Ethernet layer is initialized
    bool is_initialized();

	/// @brief Cycle comms, this issues packet read and write calls
	// std::optional<EthernetPacket> sendReceive(EthernetPacket& outgoing_packet);

private:
	/// @brief Check to see if the connection is still alive. This acts on a timeout of the last packet received
	void check_connection();

private:
	/// @brief The Teensy's static IP
	const IPAddress m_teensy_ip = { 128, 128, 128, 120 };
	/// @brief The Teensy's static gateway
	const IPAddress m_teensy_gateway = { 192, 168, 1, 1 };
	/// @brief The Teensy's static network mask
	const IPAddress m_teensy_netmask = { 255, 255, 255, 0 };
	/// @brief The Teensy's static receive port
	const uint16_t 	m_teensy_port = 35653;

	/// @brief The Jetson's static IP
	const IPAddress m_jetson_ip = { 128, 128, 128, 1 };
	/// @brief The Jetson's static receive port
	const uint16_t 	m_jetson_port = 35654;

	/// @brief The UDP server object, initialized with 50 packet buffer
	qn::EthernetUDP m_udp_server = qn::EthernetUDP(50);

	/// @brief A time in us for a minimum ethernet loop. If ethernet runs too fast, it will break and not send/recv anything. This is calculated based on a set data rate
	uint32_t m_regulation_time = 0;

	/// @brief The data rate in mbps
	uint32_t m_data_rate = 0;

	/// @brief The current connection status with the Jetson
	bool m_connected = false;

	/// @brief The last time a packet was received in microseconds
	uint32_t m_last_recv_time = 0;

	/// @brief The last time a packet was sent in microseconds
	uint32_t m_last_send_time = 0;

	/// @brief The receive packet timeout in microseconds
	const uint32_t m_connection_timeout = 500000;
	/// @brief The handshake timeout in microseconds
	const uint32_t m_handshake_timeout = 1000000;
	/// @brief The UDP warmup timeout in microseconds
	const uint32_t m_warmup_timeout = 5000000;

	/// @brief The incoming packet buffer
	EthernetPacket m_incoming = {};
	/// @brief The outgoing packet buffer
	EthernetPacket m_outgoing = {};

	/// @brief Timer for the regulation time
	Timer m_regulation_timer;
};

}	// namespace Comms