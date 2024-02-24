#include "usb_hid.hpp"

HIDLayer::HIDLayer() {}

void HIDLayer::init() {}

void HIDLayer::ping() {
	while (usb_rawhid_available()) {
		if (read()) {
			write();
			Serial.printf("Ping: %llu %llu %llu\n", m_packetsRead, m_packetsSent, m_packetsFailed);
		}
	}
}

bool HIDLayer::read() {
	int bytes_read = usb_rawhid_recv(m_incommingPacket, 0);
	if (bytes_read == PACKET_SIZE) {
		m_packetsRead++;
		return true;
	}
	else {
		return false;
	}
}

bool HIDLayer::write() {
	int bytes_sent = usb_rawhid_send(m_outgoingPacket, UINT16_MAX);
	if (bytes_sent == PACKET_SIZE) {
		m_packetsSent++;
		return true;
	}
	else {
		m_packetsFailed++;
		return false;
	}
}


