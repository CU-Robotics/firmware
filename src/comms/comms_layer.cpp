#include "comms_layer.hpp"
#include "hid_comms.hpp"

namespace Comms {

CommsLayer::CommsLayer() {
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
    // process Ethernet stack
    process_ethernet_layer();

    // process HID stack
    process_hid_layer();

    return 0;
};

void CommsLayer::queue_data(CommsData* data) {
#if defined(HIVE)
    switch (data->physical_medium) {
    case PhysicalMedium::HID:
        m_hid_payload.add(data);
        break;
    case PhysicalMedium::Ethernet:
        // if ethernet is down and it is a small enough packet, route it through HID instead
        if (!is_ethernet_connected() && data->size < HID_PACKET_PAYLOAD_SIZE) {
            m_hid_payload.add(data);
            break;
        } else if (data->size > HID_PACKET_PAYLOAD_SIZE) {
            // discard attempt to send
            printf("Attempting to re-route %s to HID but packet is too large\n", to_string(data->type_label).c_str());
            break;
        }

        m_ethernet_payload.add(data);
        break;
    default:
        throw std::runtime_error("Invalid PhysicalMedium " + std::to_string(static_cast<uint8_t>(data->physical_medium)));
    }   
#elif defined(FIRMWARE)
    switch (data->physical_medium) {
    case PhysicalMedium::HID:
        m_hid_payload.add(data);
        break;
    case PhysicalMedium::Ethernet:
        // if ethernet is down and it is a small enough packet, route it through HID instead
        if (!is_ethernet_connected() && data->size < HID_PACKET_PAYLOAD_SIZE) {
            m_hid_payload.add(data);
            break;
        } else if (data->size > HID_PACKET_PAYLOAD_SIZE) {
            // discard attempt to send
            Serial.printf("Attempting to re-route %s to HID but packet is too large\n", to_string(data->type_label).c_str());
            break;
        }

        m_ethernet_payload.add(data);
        break;
    default:
        assert(false && "Invalid PhysicalMedium");
    }
#endif
}

void CommsLayer::process_hid_layer() {
    HIDPacket hid_outgoing;
    m_hid_payload.construct_data();
    memcpy(hid_outgoing.payload(), m_hid_payload.data(), HID_PACKET_PAYLOAD_SIZE);
    std::optional<HIDPacket> hid_incoming = m_hid.sendReceive(hid_outgoing);

    if (hid_incoming.has_value()) {
        m_hid_payload.deconstruct_data((uint8_t*)hid_incoming.value().payload(), HID_PACKET_PAYLOAD_SIZE);
    }
};

void CommsLayer::process_ethernet_layer() {
    EthernetPacket eth_outgoing;
    m_ethernet_payload.construct_data();
    memcpy(eth_outgoing.payload.data, m_ethernet_payload.data(), ETHERNET_PACKET_PAYLOAD_MAX_SIZE);
    std::optional<EthernetPacket> ethernet_incoming = m_ethernet.sendReceive(eth_outgoing);

    if (ethernet_incoming.has_value()) {
        m_ethernet_payload.deconstruct_data(ethernet_incoming.value().payload.data, ETHERNET_PACKET_PAYLOAD_MAX_SIZE);
    }
};

bool CommsLayer::is_ethernet_connected() {
    return m_ethernet.is_connected();
};

bool CommsLayer::is_hid_connected() {
    // HID is always connected
    return true;
};

HiveData CommsLayer::get_hive_data() {
    return m_hive_data;
};

void CommsLayer::set_hive_data(HiveData& data) {
    m_hive_data = data;
};

FirmwareData CommsLayer::get_firmware_data() {
    return m_firmware_data;
};

void CommsLayer::set_firmware_data(FirmwareData& data) {
    m_firmware_data = data;
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