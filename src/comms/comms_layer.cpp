#include "comms_layer.hpp"
#include "ethernet_packet.hpp"
#include "hid_comms.hpp"

namespace Comms {

CommsLayer::CommsLayer() {
    m_ethernet_payload = PacketPayload(ETHERNET_PACKET_PAYLOAD_MAX_SIZE);
    m_hid_payload = PacketPayload(HID_PACKET_SIZE);
};

CommsLayer::~CommsLayer() {

};

int CommsLayer::init() {
    // Initialize the HID and Ethernet physical layers
    if (!initialize_hid()) {
        return -1;
    }
    if (!initialize_ethernet()) {
        return -1;
    }

    return 0;
};

int CommsLayer::run() {
    m_ethernet.set_outgoing_packet(m_ethernet_outgoing);
    m_ethernet.loop();

    m_hid.sendReceive(m_hid_outgoing);

    return 0;
};

void CommsLayer::queue_data(CommsData* data) {

};

HIDPacket CommsLayer::get_hid_incoming() {
    return m_hid.get_incoming_packet();
};

void CommsLayer::set_hid_outgoing(HIDPacket& packet) {
    m_hid_outgoing = packet;
};

EthernetPacket CommsLayer::get_ethernet_incoming() {
    return m_ethernet.get_incoming_packet();
};

void CommsLayer::set_ethernet_outgoing(EthernetPacket& packet) {
    m_ethernet_outgoing = packet;
};

bool CommsLayer::initialize_hid() {
    // Initialize the HID physical layer
    m_hid.init();

    return true;
};

bool CommsLayer::initialize_ethernet() {
    // Initialize the Ethernet physical layer
    m_ethernet.begin();

    return true;
};

}   // namespace Comms