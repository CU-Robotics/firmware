#pragma once

#include <Arduino.h>

#include <QNEthernet.h>
namespace qn = qindesign::network;

#include "utils/timing.hpp"				// for Timer
#include "comms/ethernet_packet.hpp"	// for EthernetPacket

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
    /// @param packet_size Total packet size in bytes to transmit (header + payload)
    /// @return True if success
	bool send_packet(EthernetPacket& packet,uint32_t packet_size = ETHERNET_PACKET_MAX_SIZE);
    /// @brief Receive a packet from Hive
    /// @param packet The packet to fill with data
    /// @return True if success
    bool recv_packet(EthernetPacket& packet);

	/// @brief Get the current connection status to Hive
    /// @return Connected or not
    bool is_connected() const;

    /// @brief Get the current initialized status of the Ethernet layer
    /// @return True if the Ethernet layer is initialized
    bool is_initialized() const;

	/// @brief Check whether physical Ethernet link is detected (cable plugged in)
	/// @return True if Ethernet link is up, false otherwise
	bool is_link_up() const;

	/// @brief Get the total number of packets successfully sent over Ethernet
	/// @return Total packets sent
	uint32_t get_packets_sent() const { return m_packets_sent; }

	/// @brief Get the total number of packets that failed to send
	/// @return Total send failures
	uint32_t get_packets_send_failed() const { return m_packets_send_failed; }

	/// @brief Get the total number of packets successfully received over Ethernet
	/// @return Total packets received
	uint32_t get_packets_received() const { return m_packets_received; }

	/// @brief Get the total number of packets that failed to receive or were dropped
	/// @return Total receive failures/drops
	uint32_t get_packets_recv_failed() const { return m_packets_recv_failed; }

	/// @brief Get the total bytes sent over Ethernet UDP
	/// @return Total bytes sent
	uint32_t get_total_bytes_sent() const { return m_total_bytes_sent; }

	/// @brief Get the total bytes received over Ethernet UDP
	/// @return Total bytes received
	uint32_t get_total_bytes_received() const { return m_total_bytes_received; }

	/// @brief Get the total size in bytes of the last packet sent
	/// @return Last sent packet size in bytes
	uint32_t get_last_send_packet_size() const { return m_last_send_packet_size; }

	/// @brief Get the total size in bytes of the last valid packet received
	/// @return Last received packet size in bytes
	uint32_t get_last_recv_packet_size() const { return m_last_recv_packet_size; }

	/// @brief Get the size in bytes of the last rejected or malformed packet
	/// @return Last error packet size in bytes (0 if none)
	int32_t get_last_recv_error_size() const { return m_last_recv_error_size; }

	/// @brief Get the timestamp of the last packet transmission in microseconds
	/// @return Timestamp in microseconds
	uint32_t get_last_send_time() const { return m_last_send_time; }

	/// @brief Get the timestamp of the last packet reception in microseconds
	/// @return Timestamp in microseconds
	uint32_t get_last_recv_time() const { return m_last_recv_time; }

	/// @brief Get the connection timeout threshold in microseconds
	/// @return Connection timeout in microseconds
	uint32_t get_connection_timeout() const { return m_connection_timeout; }

	/// @brief Get the Teensy local static IP address
	/// @return Teensy IPAddress
	IPAddress get_teensy_ip() const { return m_teensy_ip; }

	/// @brief Get the Teensy local receive port
	/// @return Teensy port number
	uint16_t get_teensy_port() const { return m_teensy_port; }

	/// @brief Get the destination Jetson static IP address
	/// @return Jetson IPAddress
	IPAddress get_jetson_ip() const { return m_jetson_ip; }

	/// @brief Get the destination Jetson receive port
	/// @return Jetson port number
	uint16_t get_jetson_port() const { return m_jetson_port; }

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
	qn::EthernetUDP m_udp_server = qn::EthernetUDP(10);

	/// @brief A time in us for a minimum ethernet loop. If ethernet runs too fast, it will break and not send/recv anything. This is calculated based on a set data rate
	uint32_t m_regulation_time = 0;

	/// @brief The data rate in mbps
	uint32_t m_data_rate = 0;

	/// @brief The current connection status with the Jetson
	bool m_connected = false;

	/// @brief The current initialized status of the Ethernet server
	bool m_initialized = false;

	/// @brief The last time a packet was received in microseconds
	uint32_t m_last_recv_time = 0;

	/// @brief The last time a packet was sent in microseconds
	uint32_t m_last_send_time = 0;

	/// @brief Total packets successfully sent
	uint32_t m_packets_sent = 0;
	/// @brief Total packets that failed to send
	uint32_t m_packets_send_failed = 0;
	/// @brief Total packets successfully received
	uint32_t m_packets_received = 0;
	/// @brief Total packets that failed to receive or were dropped
	uint32_t m_packets_recv_failed = 0;
	/// @brief Total bytes sent over UDP
	uint32_t m_total_bytes_sent = 0;
	/// @brief Total bytes received over UDP
	uint32_t m_total_bytes_received = 0;
	/// @brief Size of the last packet sent in bytes
	uint32_t m_last_send_packet_size = 0;
	/// @brief Size of the last packet received in bytes
	uint32_t m_last_recv_packet_size = 0;
	/// @brief Size of the last rejected/error packet received in bytes
	int32_t m_last_recv_error_size = 0;

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
