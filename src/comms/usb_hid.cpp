#include "usb_hid.hpp"

void SensorData::set_dr16(char* data)
{
	memcpy(raw + SENSOR_DR16_OFFSET, data, 18);
}

uint16_t CommsPacket::get_id()
{
	return *reinterpret_cast<uint16_t*>(raw + TEENSY_PACKET_ID_OFFSET);
}

uint16_t CommsPacket::get_info()
{
	return *reinterpret_cast<uint16_t*>(raw + TEENSY_PACKET_INFO_OFFSET);
}

void CommsPacket::set_id(uint16_t id)
{
	*reinterpret_cast<uint16_t*>(raw + TEENSY_PACKET_ID_OFFSET) = id;
}

void CommsPacket::set_info(uint16_t info)
{
	*reinterpret_cast<uint16_t*>(raw + TEENSY_PACKET_INFO_OFFSET) = info;
}

void CommsPacket::get_target_state(float state[STATE_LEN][3])
{
	memcpy(state, raw + KHADAS_PACKET_TSTATE_OFFSET, sizeof(float) * STATE_LEN * 3);
}

void CommsPacket::get_ref_draw_data(char** draw_data) {}

void CommsPacket::set_time(double time)
{
	*reinterpret_cast<double*>(raw + TEENSY_PACKET_TIME_OFFSET) = time;
}

void CommsPacket::set_estimated_state(float state[STATE_LEN][3])
{
	memcpy(raw + TEENSY_PACKET_ESTATE_OFFSET, state, sizeof(float) * STATE_LEN * 3);
}

void CommsPacket::set_sensor_data(SensorData* sensor_data)
{
	memcpy(raw + TEENSY_PACKET_SENSOR_OFFSET, sensor_data.raw, sizeof(SensorData));
}

void CommsPacket::set_ref_data() {}


HIDLayer::HIDLayer() {}

void HIDLayer::init() { Serial.println("Starting HID layer"); }

void HIDLayer::ping() {
	while (usb_rawhid_available()) {
		if (read()) {
			if (!write())
				Serial.printf("Failed to send ping %llu\n", m_packetsSent);
		}
	}
}

void HIDLayer::print() {
	for (int i = 0; i < PACKET_SIZE + 1; i++)
		Serial.printf("%.2x ", m_outgoingPacket.raw[i]);

	Serial.println();
}

bool HIDLayer::read() {
	int bytes_read = usb_rawhid_recv(m_incommingPacket.raw, 0);
	if (bytes_read == PACKET_SIZE) {
		m_packetsRead++;
		return true;
	}
	else {
		return false;
	}
}

bool HIDLayer::write() {
	int bytes_sent = usb_rawhid_send(m_outgoingPacket.raw, UINT16_MAX);
	if (bytes_sent == PACKET_SIZE) {
		m_packetsSent++;
		((long long unsigned*)m_outgoingPacket.raw)[0] = m_packetsSent;
		return true;
	}
	else {
		m_packetsFailed++;
		return false;
	}
}


