#include "usb_hid.hpp"

uint16_t CommsPacket::get_id() {
	// c++ moment
	char* x = raw;
	return *reinterpret_cast<uint16_t*>(x + TEENSY_PACKET_ID_OFFSET);
}

uint16_t CommsPacket::get_info() {
	return *reinterpret_cast<uint16_t*>(raw + TEENSY_PACKET_INFO_OFFSET);
}

void CommsPacket::set_id(uint16_t id) {
	// c++ moment
	char* x = raw;
	*reinterpret_cast<uint16_t*>(x + TEENSY_PACKET_ID_OFFSET) = id;
}

void CommsPacket::set_info(uint16_t info) {
	*reinterpret_cast<uint16_t*>(raw + TEENSY_PACKET_INFO_OFFSET) = info;
}

void CommsPacket::get_target_state(float state[STATE_LEN][3]) {
	memcpy(state, raw + KHADAS_PACKET_TSTATE_OFFSET, sizeof(float) * STATE_LEN * 3);
}

void CommsPacket::get_ref_draw_data(char** draw_data) {}

void CommsPacket::set_time(double time) {
	*reinterpret_cast<double*>(raw + TEENSY_PACKET_TIME_OFFSET) = time;
}

void CommsPacket::set_estimated_state(float state[STATE_LEN][3]) {
	memcpy(raw + TEENSY_PACKET_ESTATE_OFFSET, state, sizeof(float) * STATE_LEN * 3);
}

void CommsPacket::set_sensor_data(SensorData* sensor_data) {
	memcpy(raw + TEENSY_PACKET_SENSOR_OFFSET, sensor_data->raw, sizeof(SensorData));
}

void CommsPacket::set_ref_data() {}


HIDLayer::HIDLayer() {}

void HIDLayer::init() { Serial.println("Starting HID layer"); }

void HIDLayer::ping() {
	// loop until the packet buffer is empty (this is only ever 1 packet large)
	while (usb_rawhid_available()) {
		// attempt to read
		if (read()) {
			// if we read, attempt to write

			// set the packet to be written's ID
			m_outgoingPacket.set_id((uint16_t)m_packetsSent);
			if (!write())
				Serial.printf("Failed to send ping %llu\n", m_packetsSent);
		}
	}
}

void HIDLayer::print() {
	for (unsigned int i = 0; i < COMMS_PACKET_SIZE + 1; i++)
		Serial.printf("%.2x ", m_outgoingPacket.raw[i]);

	Serial.println();
}

bool HIDLayer::read() {
	// attempt to read a full packet
	// this has no timeout
	int bytes_read = usb_rawhid_recv(m_incommingPacket.raw, 0);
	if (bytes_read == COMMS_PACKET_SIZE) {
		// increment total number of packets read and return success
		m_packetsRead++;
		return true;
	}
	else {
		return false;
	}
}

bool HIDLayer::write() {
	// attempt to write a full packet
	// this has a timeout, which is set to it's max value
	int bytes_sent = usb_rawhid_send(m_outgoingPacket.raw, UINT16_MAX);
	if (bytes_sent == COMMS_PACKET_SIZE) {
		// increment total number of packets sent and return success
		m_packetsSent++;
		return true;
	} 
	else {
		m_packetsFailed++;
		return false;
	}
}
