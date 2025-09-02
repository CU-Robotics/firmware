#include "comms_layer.hpp"

namespace Comms {

CommsLayer::CommsLayer() {
    logger.printf(LogDestination::Serial, "CommsLayer: constructed\n");
};

CommsLayer::~CommsLayer() {
    logger.printf(LogDestination::Serial, "CommsLayer: destructed\n");
};

int CommsLayer::init() {
    logger.printf(LogDestination::Serial, "CommsLayer: initializing\n");
    
    // hid failing is a fatal error
    bool hid_init = initialize_hid();
    if (!hid_init) {
        logger.printf(LogDestination::Serial, "CommsLayer: HIDComms init failed\n");
        return -1;
    }

    // ethernet init failing is not a fatal error
    bool ethernet_init = initialize_ethernet();
    if (!ethernet_init) {
        logger.printf(LogDestination::Serial, "CommsLayer: EthernetComms init failed\n");
    }

    logger.printf(LogDestination::Serial, "CommsLayer: initialized\n");

    return 0;
};

int CommsLayer::run() {
    // read packets from the physical layers
    receive_packets();

    // write packets to the physical layers
    send_packets();

    return 0;
};

void CommsLayer::queue_data(CommsData* data) {
    switch (data->physical_medium) {
    case PhysicalMedium::HID:
        if (!is_hid_connected()) {
            // discard attempt to send
            logger.printf(LogDestination::Serial, "Attempting to re-route %s to HID but HID is not connected\n", to_string(data->type_label).c_str());
            break;
        }
        m_hid_payload.add(data);
        break;
    case PhysicalMedium::Ethernet:
        // if ethernet is down and it is a small enough packet, route it through HID instead
        if (!is_ethernet_connected() && data->size < HID_PACKET_PAYLOAD_SIZE) {
            m_hid_payload.add(data);
            break;
        } else if (!is_ethernet_connected() && data->size > HID_PACKET_PAYLOAD_SIZE) {
            // discard attempt to send
            logger.printf(LogDestination::Serial, "Attempting to re-route %s to HID but packet is too large: ", to_string(data->type_label).c_str(), data->size);
            logger.printf(LogDestination::Serial, "(%d / %d)\n", data->size, HID_PACKET_PAYLOAD_SIZE);
            break;
        }

        m_ethernet_payload.add(data);
        break;
    default:
        assert(false && "Invalid PhysicalMedium");
    }
};

void CommsLayer::send_packets() {
    // prepare and send a HID packet
    m_hid_payload.construct_data();
    memcpy(m_hid_outgoing.payload(), m_hid_payload.data(), m_hid_payload.get_max_size());
    m_hid.send_packet(m_hid_outgoing);
    
    // prepare and send an ethernet packet
    m_ethernet_payload.construct_data();
    memcpy(m_ethernet_outgoing.payload(), m_ethernet_payload.data(), m_ethernet_payload.get_max_size());
    m_ethernet.send_packet(m_ethernet_outgoing);
};

void CommsLayer::receive_packets() {
    // defaulted to true so tests can run without physical layers
    bool hid_recv = true;
    bool ethernet_recv = true;
    
    // receive packets from the appropriate physical layer
    if (m_hid.is_initialized()) {
        hid_recv = m_hid.recv_packet(m_hid_incoming);
    }
    if (m_ethernet.is_initialized()) {
        ethernet_recv = m_ethernet.recv_packet(m_ethernet_incoming);
    }

    // process packets
    if (hid_recv) {
        m_hid_payload.deconstruct_data(m_hid_incoming.payload(), m_hid_payload.get_max_size());
    }
    if (ethernet_recv) {
        m_ethernet_payload.deconstruct_data(m_ethernet_incoming.payload(), m_ethernet_payload.get_max_size());
    }    
};

bool CommsLayer::is_ethernet_connected() {
    return m_ethernet.is_initialized() && m_ethernet.is_connected();
};

bool CommsLayer::is_hid_connected() {
    return m_hid.is_initialized() && m_hid.is_connected();
};

void CommsLayer::clear_outgoing_buffers() {
    m_hid_payload.clear_queues();
    m_ethernet_payload.clear_queues();
};

EthernetPacket CommsLayer::get_ethernet_outgoing() {
    return m_ethernet_outgoing;
};

HIDPacket CommsLayer::get_hid_outgoing() {
    return m_hid_outgoing;
};

void CommsLayer::set_ethernet_incoming(EthernetPacket&& packet) {
    m_ethernet_incoming = packet;
};

void CommsLayer::set_hid_incoming(HIDPacket&& packet) {
    m_hid_incoming = packet;
};

HiveData& CommsLayer::get_hive_data() {
    return m_hive_data;
};

void CommsLayer::set_hive_data(HiveData& data) {
    m_hive_data = data;
};

FirmwareData& CommsLayer::get_firmware_data() {
    return m_firmware_data;
};

void CommsLayer::set_firmware_data(FirmwareData& data) {
    m_firmware_data = data;
};

bool CommsLayer::initialize_hid() {
    // Initialize the HID physical layer
    m_hid.init();

    return true;
};

bool CommsLayer::initialize_ethernet() {
    // Initialize the Ethernet physical layer
    if (!m_ethernet.init()) {
        logger.println(LogDestination::Serial, "Ethernet initialization failed");
        return false;
    }
    
    logger.println(LogDestination::Serial, "Ethernet initialized successfully");

    return true;
};

}   // namespace Comms