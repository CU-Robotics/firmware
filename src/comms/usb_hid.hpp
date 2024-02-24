#ifndef USB_HID_HPP
#define USB_HID_HPP

#include "Arduino.h"
#include "usb_rawhid.h"

constexpr int PACKET_SIZE = 1023;

class HIDLayer
{
public:
	HIDLayer();
	
	void init();

	void ping();

private:
	bool read();
	bool write();

private:
	char m_incommingPacket[PACKET_SIZE + 1] = {0};
	char m_outgoingPacket[PACKET_SIZE + 1] = {0};

	long long unsigned m_packetsRead = 0;
	long long unsigned m_packetsSent = 0;
	long long unsigned m_packetsFailed = 0;
};

#endif // end USB_HID_HPP
