#include "usb_hid.hpp"

void CommsPacket::set_time(double time)
{
    memcpy(raw + PACKET_TIME_OFFSET, &time, sizeof(double));
}

void CommsPacket::set_state(float state[STATE_LEN][3])
{
    memcpy(raw + PACKET_STATE_OFFSET, state, sizeof(float) * STATE_LEN * 3);
}

void CommsPacket::set_dr16(char data[DR16_PACKET_SIZE])
{
    memcpy(raw + PACKET_DR16_OFFSET, data, sizeof(char) * DR16_PACKET_SIZE);
}

HIDLayer::HIDLayer() {}

void HIDLayer::init() { Serial.println("Starting HID layer"); }

void HIDLayer::ping()
{
    while (usb_rawhid_available())
    {
        if (read())
        {
            write();
            // Serial.printf("Ping: %llu %llu %llu\n", m_packetsRead, m_packetsSent, m_packetsFailed);
        }
    }
}

void HIDLayer::print()
{
    for (int i = 0; i < PACKET_SIZE + 1; i++)
        Serial.printf("%.2x ", m_outgoingPacket.raw[i]);

    Serial.println();
}

bool HIDLayer::read()
{
    int bytes_read = usb_rawhid_recv(m_incommingPacket.raw, 0);
    if (bytes_read == PACKET_SIZE)
    {
        m_packetsRead++;
        return true;
    }
    else
    {
        return false;
    }
}

bool HIDLayer::write()
{
    int bytes_sent = usb_rawhid_send(m_outgoingPacket.raw, UINT16_MAX);
    if (bytes_sent == PACKET_SIZE)
    {
        m_packetsSent++;
        return true;
    }
    else
    {
        m_packetsFailed++;
        return false;
    }
}
