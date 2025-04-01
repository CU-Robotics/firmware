#include "hid_comms.hpp"

namespace Comms {

HIDComms::HIDComms() {}

void HIDComms::init() { Serial.println("HIDComms: Starting HID layer"); }

std::optional<HIDPacket> HIDComms::sendReceive(HIDPacket& outgoing_packet) {
    // loop until the packet buffer is empty (this is only ever 1 packet large)
    while (usb_rawhid_available()) {
        HIDPacket incoming_packet;
        
        // attempt to read
        if (read(incoming_packet)) {
            // if we read, attempt to write
            if (!write(outgoing_packet))
                Serial.printf("HIDComms: Failed to send: %llu\n", m_packetsSent);

            return m_incomingPacket;
        }
    }

    return {};
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

bool HIDComms::read(HIDPacket& incoming_packet) {
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

bool HIDComms::write(HIDPacket& outgoing_packet) {
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

}  // namespace Comms