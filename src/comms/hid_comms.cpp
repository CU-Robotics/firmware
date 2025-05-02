#include "hid_comms.hpp"
#include "logger.hpp"

namespace Comms {

HIDComms::HIDComms() {}

void HIDComms::init() { 
    logger.println(LogDestination::Serial, "HIDComms: Starting HID layer");
}

bool HIDComms::recv_packet(HIDPacket& incoming_packet) {
    // attempt to read a full packet
    // this has no timeout
    int bytes_read = usb_rawhid_recv(incoming_packet.data_start(), 0);
    if (bytes_read == HID_PACKET_MAX_SIZE) {
        // increment total number of packets read and return success
        m_packetsRead++;
        m_incomingPacket = incoming_packet;
        m_received_last = true;
        return true;
    } else {
        m_received_last = false;
        return false;
    }
}

bool HIDComms::send_packet(HIDPacket& outgoing_packet) {
    // check if the last packet was received
    if (!m_received_last) {
        return false;
    }
    
    // reset the received last flag
    m_received_last = false;
    
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

bool HIDComms::is_initialized() const {
    return true;
}

void HIDComms::print_outgoing() {
    logger.println(LogDestination::Serial, "HIDComms: Outgoing packet:");
    for (unsigned int i = 0; i < HID_PACKET_MAX_SIZE; i++)
        logger.printf(LogDestination::Serial, "%.2x ", m_outgoingPacket.data_start()[i]);

    logger.println(LogDestination::Serial );
}

void HIDComms::print_incoming() {
    logger.println(LogDestination::Serial, "HIDComms: Incoming packet:");
    for (unsigned int i = 0; i < HID_PACKET_MAX_SIZE; i++)
        logger.printf(LogDestination::Serial, "%.2x ", m_incomingPacket.data_start()[i]);

    logger.println(LogDestination::Serial);
}

}  // namespace Comms