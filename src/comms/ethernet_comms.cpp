#include "ethernet_comms.hpp"

namespace Comms {

bool EthernetComms::begin(uint32_t data_rate) {
	// begin the ethernet library and verify the status of the line
	uint8_t ethernet_status = qn::Ethernet.begin(m_teensy_ip, m_teensy_netmask, m_teensy_gateway);
	if (!ethernet_status) {
		Serial.printf("Failed to start Ethernet!\n");

		// check if this teensy even has ethernet hardware support
		qn::EthernetHardwareStatus hw_status = qn::Ethernet.hardwareStatus();
		if (hw_status != qn::EthernetHardwareStatus::EthernetTeensy41)
			Serial.printf("Teensy Ethernet hardware not present!\n");

		return false;
	} else {
	#ifdef COMMS_DEBUG
		Serial.printf("Ethernet started!\n");
		Serial.printf("IP: %u.%u.%u.%u:%u\n", qn::Ethernet.localIP()[0], qn::Ethernet.localIP()[1], qn::Ethernet.localIP()[2], qn::Ethernet.localIP()[3], m_teensy_port);
	#endif
	}

	// begin the UDP server
	uint8_t udp_status = m_udp_server.begin(m_teensy_port);
	// this failing is extremely bad. Either the Teensy is out of memory, or the hardware is broken
	if (!udp_status) {
		Serial.printf("UDP server failed to start!\n");

		return false;
	} else {
	#ifdef COMMS_DEBUG
		Serial.printf("UDP server started!\n");
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
	while (!send_packet(m_outgoing)) {
		qn::Ethernet.loop();
		delayMicroseconds(m_regulation_time);
	}

	// ethernet is now setup and udp server is online
	Serial.printf("Ethernet online!\n");

	return true;
}

void EthernetComms::loop() {
	// call the library's loop function
	// this progresses the ethernet stack allowing new data in/out
	qn::Ethernet.loop();

	// send the outgoing packet
	send_packet(m_outgoing);

	// receive the incoming packet
	recv_packet(&m_incoming);

	// if we detect a handshake request from Jetson, respond
	if (m_incoming.header.type == Comms::EthernetPacketType::HANDSHAKE) {
		Serial.printf("Handshake received...\n");
		connect_jetson();
		Serial.printf("Handshake finished!\n");
	}

	// check whether the connection is still alive
	check_connection();

	return;
}

EthernetPacket* const EthernetComms::get_incoming_packet() {
	return &m_incoming;
}

EthernetPacket* const EthernetComms::get_outgoing_packet() {
	return &m_outgoing;
}

bool EthernetComms::is_connected() const {
	return m_connected;
}

const EthernetStatus& EthernetComms::get_status() const {
	return m_status;
}

uint32_t EthernetComms::get_regulation_time() const {
	return m_regulation_time;
}

int EthernetComms::connect_jetson() {
	// clear the packet buffers
	m_incoming.clear();
	m_outgoing.clear();

	// assemble the packet to be a HANDSHAKE packet with the ACK flag set
	m_outgoing.header.type = Comms::EthernetPacketType::HANDSHAKE;
	m_outgoing.header.flags = Comms::EthernetPacketFlags::ACK;

	uint32_t handshake_start = micros();

	uint32_t last_loop = micros();

	bool handshake = false;

	// loop until handshake is complete or a timeout is reached
	while (!handshake) {
		// regulate time
		// this is to prevent the Teensy from running too fast and overloading the hardware
		if (micros() - last_loop < m_regulation_time)
			continue;

		// process the ethernet stack
		qn::Ethernet.loop();

		// if we havnt received a packet, keep trying
		if (!recv_packet(&m_incoming))
			continue;

		// if we received a packet that is not a handshake, call it done
		// this indicates that khadas received our ACK and is now sending normal data
		// note that we will lose the first packet sent after a handshake, this is an acceptable loss
		if (m_incoming.header.type != Comms::EthernetPacketType::HANDSHAKE) {
			handshake = true;

			// reset the buffers to prevent infinite running this handshake protocol
			m_incoming.clear();
			m_outgoing.clear();

			// reset the comms status
			m_status.packets_received = 0;
			m_status.packets_received_failed = 0;
			m_status.packets_sent = 0;
			m_status.packets_sent_failed = 0;

			// log the handshake time
			m_status.handshake_time = micros();

		#ifdef COMMS_DEBUG
			Serial.printf("Handshake took %lu us\n", m_status.handshake_time - handshake_start);
		#endif

			// return success
			return 0;
		}

		// send our ACK packet
		send_packet(m_outgoing);

		// check to see if this handshake is taking too long and time it out to prevent Teensy from being soft locked
		// this does not prevent Hive from just spamming the Teensy with handshake requests, although that should be impossible
		// I have yet to see a handshake fail, so consider this a sanity check if nothing else
		if (micros() - handshake_start >= m_handshake_timeout) {
			Serial.printf("Handshake failed: timeout!\n");
			// return failure
			return -1;
		}
	}

	// return failure (program never reaches this point)
	return -1;
}

bool EthernetComms::send_packet(EthernetPacket& packet) {
	// attempt to send the packet to the Jetson
	bool send_status = m_udp_server.send(m_jetson_ip, m_jetson_port, packet.data(), Comms::ETHERNET_PACKET_MAX_SIZE);
	if (!send_status) {
		// log the fail, this almost always happens when the udp server is not "warmed up"
		m_status.packets_sent_failed++;
	#if defined(COMMS_DEBUG)
		Serial.printf("Comms: Send fail: %lu\n", m_status.packets_sent_failed);
	#endif
		return false;
	} else {
		// log the success
		m_status.packets_sent++;
		return true;
	}

	return false;
}

bool EthernetComms::recv_packet(EthernetPacket* packet) {
	// attempt to receive a packet
	// parsePacket returns the current size of the RX buffer
	int current_buffer_size = m_udp_server.parsePacket();
	
	// if this buffer data is the size of a packet, process it
	if (current_buffer_size == -1) {
		// no packet to be read
		return false;
	} else if (current_buffer_size != Comms::ETHERNET_PACKET_MAX_SIZE) {
		// half-read, log as a failure
		m_status.packets_received_failed++;
	#if defined(COMMS_DEBUG)
		Serial.printf("Comms: Recv fail: %d %lu\n", current_buffer_size, m_status.packets_received_failed);
	#endif
		return false;
	} else {
		// log the success
		m_status.packets_received++;

		// grab the data pointer
		const uint8_t* packet_data = m_udp_server.data();
		// this should never happen, but sanity check
		if (packet_data == NULL) {
		#if defined(COMMS_DEBUG)
			Serial.printf("Comms: Recv data NULL\n");
		#endif
		}

		// copy the data of the buffer into the receive packet
		memcpy(packet, m_udp_server.data(), current_buffer_size);

		// log this packet as the last packet received
		m_last_recv_time = micros();

		return true;
	}

	return false;
}

void EthernetComms::check_connection() {
	// if the last packet was received too long ago, timeout the connection
	if (micros() - m_last_recv_time > m_connection_timeout) {
	#ifdef COMMS_DEBUG
		// this check ensures this is only printed once
		if (m_connected)
			Serial.printf("Connection lost!\n");
	#endif
		// mark the connection as disconnected
		m_connected = false;
	} else {
		m_connected = true;
	}

	return;
}

}	// namespace Comms