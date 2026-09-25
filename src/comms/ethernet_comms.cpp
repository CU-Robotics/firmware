#include "ethernet_comms.hpp"
#include "utils/system_log.hpp"

namespace Comms {

bool EthernetComms::init(uint32_t data_rate) {
	// begin the ethernet library and verify the status of the line
	uint8_t ethernet_status = qn::Ethernet.begin(m_teensy_ip, m_teensy_netmask, m_teensy_gateway);
	if (!ethernet_status) {
		SystemLog.warn(Subsystem::COMMS,"EthernetComms: Failed to start Ethernet!\n");

		// check if this teensy even has ethernet hardware support
		qn::EthernetHardwareStatus hw_status = qn::Ethernet.hardwareStatus();
		if (hw_status != qn::EthernetHardwareStatus::EthernetTeensy41)
			SystemLog.warn(Subsystem::COMMS,"EthernetComms: Teensy Ethernet hardware not present!\n");

		return false;
	} else {
	#ifdef COMMS_DEBUG
		SystemLog.info(Subsystem::COMMS,"EthernetComms: Ethernet started!\n");
		SystemLog.info(Subsystem::COMMS,"EthernetComms: IP: %u.%u.%u.%u:%u\n", qn::Ethernet.localIP()[0], qn::Ethernet.localIP()[1], qn::Ethernet.localIP()[2], qn::Ethernet.localIP()[3], m_teensy_port);
	#endif
	}

	// begin the UDP server
	uint8_t udp_status = m_udp_server.begin(m_teensy_port);
	// this failing is extremely bad. Either the Teensy is out of memory, or the hardware is broken
	if (!udp_status) {
		SystemLog.warn(Subsystem::COMMS,"EthernetComms: UDP server failed to start!\n");

		return false;
	} else {
	#ifdef COMMS_DEBUG
		SystemLog.info(Subsystem::COMMS,"EthernetComms: UDP server started!\n");
	#endif
	}

	// assign the data rate
	m_data_rate = data_rate;

	// calculate the regulation time
	// dt = (packet_size * 8 bits) / (data_rate)
	// this regulation time is generally accurate to the nearest 0.01 mbps
	float regulation_time = (Comms::ETHERNET_PACKET_MAX_SIZE * 8.f) / (float)(data_rate);
	// round up to the nearest microsecond
	m_regulation_time = (uint32_t)ceil(regulation_time);

	// warm up the UDP server
	// when the server first starts up, there is a time period of when it establishes some sort of a connection
	// this just waits until our send commands return success (the line is ready)
	uint32_t warmup_start = micros();
	while (!send_packet(m_outgoing)) {
		qn::Ethernet.loop();
		delayMicroseconds(m_regulation_time);

		// If the warmup takes too long, timeout
		// this does not mean the ethernet is not working, just that the server never got a response from Linux
		// this is generally a problem with the Linux side, not the Teensy
		// (the ethernet cable might be disconnected or the Linux side is not running)
		if (micros() - warmup_start >= m_warmup_timeout) {
			SystemLog.warn(Subsystem::COMMS,"EthernetComms: UDP server warmup failed!\n");
			SystemLog.warn(Subsystem::COMMS,"EthernetComms: Is the ethernet cable connected?\n");

			return false;
		}

	}

	// ethernet is now setup and udp server is online
	SystemLog.info(Subsystem::COMMS,"EthernetComms: Ethernet online!\n");

	// set the initialized flag
	m_initialized = true;

	// reset stats post-warmup
	m_packets_sent = 0;
	m_packets_send_failed = 0;
	m_packets_received = 0;
	m_packets_recv_failed = 0;
	m_total_bytes_sent = 0;
	m_total_bytes_received = 0;
	m_last_send_packet_size = 0;
	m_last_recv_packet_size = 0;
	m_last_recv_error_size = 0;

	return true;
}
bool EthernetComms::send_packet(EthernetPacket& packet,uint32_t packet_size) {
    if (packet_size < PACKET_HEADER_SIZE ||
        packet_size > ETHERNET_PACKET_MAX_SIZE) {
#if defined(COMMS_DEBUG)
        SystemLog.info(Subsystem::COMMS, "EthernetComms: Packet size incorrect\n");
#endif
        m_packets_send_failed++;
        return false;
    }
	// update the connection status if needed
    check_connection();

    m_last_send_time = micros();
    m_last_send_packet_size = packet_size;
    m_regulation_timer.start();

    const bool sent = m_udp_server.send(m_jetson_ip,m_jetson_port,packet.data_start(),packet_size);

    if (sent) {
        m_packets_sent++;
        m_total_bytes_sent += packet_size;
    } else {
        m_packets_send_failed++;
#if defined(COMMS_DEBUG)
        SystemLog.info(Subsystem::COMMS,"EthernetComms: Send fail\n");
#endif
    }

    return sent;
}

bool EthernetComms::recv_packet(EthernetPacket& packet) {
	// update the connection status if needed
	check_connection();
	
	// attempt to receive a packet
	// parsePacket returns the current size of the RX buffer
	int current_buffer_size = m_udp_server.parsePacket();
	
	// if this buffer data is the size of a packet, process it
	if (current_buffer_size == -1) {
		// no packet to be read
		return false;
	} else if (current_buffer_size != Comms::ETHERNET_PACKET_MAX_SIZE) {
		// half-read, log as a failure
		m_packets_recv_failed++;
		m_last_recv_error_size = current_buffer_size;
	#if defined(COMMS_DEBUG)
		SystemLog.info(Subsystem::COMMS,"EthernetComms: Recv fail: %d\n", current_buffer_size);
	#endif
		return false;
	} else {
		// grab the data pointer
		const uint8_t* packet_data = m_udp_server.data();
		// this should never happen, but sanity check
		if (packet_data == NULL) {
		#if defined(COMMS_DEBUG)
			SystemLog.info(Subsystem::COMMS,"EthernetComms: Recv data NULL\n");
		#endif
		}

		// copy the data of the buffer into the receive packet
		memcpy(packet.data_start(), m_udp_server.data(), current_buffer_size);

		// log this packet as the last packet received
		m_last_recv_time = micros();
		m_last_recv_packet_size = current_buffer_size;
		m_packets_received++;
		m_total_bytes_received += current_buffer_size;

		return true;
	}

	return false;
}

bool EthernetComms::is_connected() const {
    return m_connected;
}

bool EthernetComms::is_initialized() const {
	return m_initialized;
}

bool EthernetComms::is_link_up() const {
	return qn::Ethernet.linkStatus() == qn::EthernetLinkStatus::LinkON;
}

void EthernetComms::check_connection() {
	// if the last packet was received too long ago, timeout the connection
	if (micros() - m_last_recv_time > m_connection_timeout) {
		// this check ensures this is only printed once
		if (m_connected) SystemLog.warn(Subsystem::COMMS,"EthernetComms: Connection lost!\n");
		// mark the connection as disconnected
		m_connected = false;
	} else {
		m_connected = true;
	}

	return;
}

}	// namespace Comms
