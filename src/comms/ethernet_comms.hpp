#pragma once

#include <Arduino.h>

// QNEthernet has warnings that are not fixable (-Wattributes)
// This is a useful warning so we dont want to permanently disable it
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#include <QNEthernet.h>
namespace qn = qindesign::network;
#pragma GCC diagnostic pop

#include "ethernet_data.hpp"

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

	/// @brief Initialize Ethernet comms and start the UDP connection
	/// @param data_rate (optional) The data rate in mbps
	/// @return True for success
	bool begin(uint32_t data_rate = 95);

	/// @brief Cycle comms, this issues packet read and write calls
	void loop();

public:
	/// @brief Get a pointer to the incoming packet
	/// @return The pointer to the internal incoming packet
	/// @note Pointer is const, the data is not; you shouldn't change the pointer itself
	EthernetPacket* const get_incoming_packet();

	/// @brief Get a pointer to the outgoing packet
	/// @return The pointer to the internal outgoing packet
	/// @note Pointer is const, the data is not; you shouldn't change the pointer itself
	EthernetPacket* const get_outgoing_packet();

	/// @brief Get the connection status with the Jetson
	/// @return True if connected
	bool is_connected() const;

	/// @brief Get the current status of the comms
	/// @return The current status of the comms
	const EthernetStatus& get_status() const;

	/// @brief Get the regulation time of the comms
	/// @return The regulation time in microseconds
	uint32_t get_regulation_time() const;

private:
	/// @brief Attempt a handshake with Jetson. This only runs if Jetson requests a handshake
	/// @note This is a blocking call, but has a timeout
	/// @return -1 for error, 0 for success
	int connect_jetson();

	/// @brief Send a packet to the Jetson
	/// @param packet The packet reference to send
	/// @return True for success
	/// @note If this returns false, the packet contents are unknown
	bool send_packet(EthernetPacket& packet);

	/// @brief Receive a packet from the Jetson
	/// @param packet The receiving packet destination
	/// @return True for success
	/// @note If this returns false, the packet destination's contents are unchanged
	bool recv_packet(EthernetPacket* packet);

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

	/// @brief The receive packet timeout in microseconds
	const uint32_t m_connection_timeout = 500000;
	/// @brief The handshake timeout in microseconds
	const uint32_t m_handshake_timeout = 1000000;

	/// @brief The current status of the comms
	EthernetStatus m_status = {};

	/// @brief The incoming packet buffer
	EthernetPacket m_incoming = {};
	/// @brief The outgoing packet buffer
	EthernetPacket m_outgoing = {};
};

}	// namespace Comms