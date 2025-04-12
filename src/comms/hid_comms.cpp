#include "hid_comms.hpp"

namespace Comms {

HIDComms::HIDComms() {}

void HIDComms::init() { 
    Serial.println("HIDComms: Starting HID layer"); 
}

bool HIDComms::recv_packet(HIDPacket& incoming_packet) {
    // attempt to read a full packet
    // this has no timeout
    int bytes_read = usb_rawhid_recv(incoming_packet.data_start(), 0);
    if (bytes_read == HID_PACKET_MAX_SIZE) {
        // increment total number of packets read and return success
        m_packetsRead++;
        m_incomingPacket = incoming_packet;
        return true;
    } else {
        return false;
    }
}

bool HIDComms::send_packet(HIDPacket& outgoing_packet) {
    // verify that the first byte is set to something (0xff)
    // prevents a weird comms issue where the whole packet is shifted left by one byte if the first byte is not ever set
    outgoing_packet.header.SOF = 0xff;

    // attempt to write a full packet
    // this has no timeout
    int bytes_sent = usb_rawhid_send(outgoing_packet.data_start(), 0);
    if (bytes_sent == HID_PACKET_MAX_SIZE) {
        // increment total number of packets sent and return success
        m_packetsSent++;
        m_outgoingPacket = outgoing_packet;
        return true;
    } else {
        m_packetsFailed++;
        return false;
    }
}

bool HIDComms::is_connected() const {
    return true;
}

bool HIDComms::is_initialized() {
    return true;
}

void HIDComms::print_outgoing() {
    Serial.println("HIDComms: Outgoing packet:");
    for (unsigned int i = 0; i < HID_PACKET_MAX_SIZE; i++)
        Serial.printf("%.2x ", m_outgoingPacket.data_start()[i]);

    Serial.println();
}

void HIDComms::print_incoming() {
    Serial.println("HIDComms: Incoming packet:");
    for (unsigned int i = 0; i < HID_PACKET_MAX_SIZE; i++)
        Serial.printf("%.2x ", m_incomingPacket.data_start()[i]);

    Serial.println();
}

}  // namespace Comms