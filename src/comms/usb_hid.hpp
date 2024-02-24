#ifndef USB_HID_HPP
#define USB_HID_HPP

#include "Arduino.h"
#include "usb_rawhid.h"
#include "../controls/state.hpp"
#include "../sensors/dr16.hpp"

constexpr int PACKET_SIZE = 1023;
constexpr int PACKET_TIME_OFFSET = 0x10;
constexpr int PACKET_STATE_OFFSET = 0x100;
constexpr int PACKET_DR16_OFFSET = 0x300;

struct CommsPacket
{
    char raw[PACKET_SIZE + 1] = {0};

    void set_time(double time);
    void set_state(float state[STATE_LEN][3]);
    void set_dr16(char data[DR16_PACKET_SIZE]);
};

class HIDLayer
{
public:
    HIDLayer();

    void init();

    void ping();

    void print();

    inline CommsPacket *get_incommming() { return &m_incommingPacket; }
    inline CommsPacket *get_outgoing() { return &m_outgoingPacket; }

private:
    bool read();
    bool write();

private:
    CommsPacket m_incommingPacket{};
    CommsPacket m_outgoingPacket{};

    long long unsigned m_packetsRead = 0;
    long long unsigned m_packetsSent = 0;
    long long unsigned m_packetsFailed = 0;
};

#endif // end USB_HID_HPP