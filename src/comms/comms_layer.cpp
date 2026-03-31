#include "comms_layer.hpp"
#include "comms/data/configuration_status_data.hpp"
#include "comms/data/sendable.hpp"


/// @brief This resets the whole processor and kicks it back to program entry (teensy4/startup.c)
/// @param void specify no arguments (needed in C)
/// @note Dont abuse this function, it is not to be used lightly
extern "C" void reset_teensy(void) {
    // Register information found in the NXP IM.XRT 1060 reference manual
    SRC_GPR5 = 0x0BAD00F1;
    // Register information found in the Arm-v7-m reference manual
    SCB_AIRCR = 0x05FA0004;
    // loop to catch execution while the reset occurs
    while (1);
}

namespace Comms {

CommsLayer::CommsLayer() {
    Serial.printf("CommsLayer: constructed\n");
};

CommsLayer::~CommsLayer() {
    Serial.printf("CommsLayer: destructed\n");
};

int CommsLayer::init() {
    Serial.printf("CommsLayer: initializing\n");
    
    // hid failing is a fatal error
    bool hid_init = initialize_hid();
    if (!hid_init) {
        Serial.printf("CommsLayer: HIDComms init failed\n");
        return -1;
    }

    // ethernet init failing is not a fatal error
    bool ethernet_init = initialize_ethernet();
    if (!ethernet_init) {
        Serial.printf("CommsLayer: EthernetComms init failed\n");
    }

    Serial.printf("CommsLayer: initialized\n");

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
            Serial.printf("Attempting to re-route %s to HID but HID is not connected\n", to_string(data->type_label).c_str());
            break;
        }
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

void CommsLayer::configure() {
    int time = millis();
    while (!m_hive_data.config.config_start.num_config_sections != 0) {
        Serial.printf("Waiting for config start packet... time since start: %d ms\n", millis() - time);
        run();
        config_loop_timer.delay_micros(5000);
    }
    Serial.printf("Config start packet received, expecting %d config sections\n", m_hive_data.config.config_start.num_config_sections);

    Sendable<ConfigurationStatusData> config_status_sendable;
    while(!m_hive_data.config.is_configured()) {
        config_status_sendable.data.ready_for_config = 1;
        config_status_sendable.send_to_comms();
        run();
        Serial.printf("Config: received %d of %d sections\n", m_hive_data.config.num_sections_received, m_hive_data.config.config_start.num_config_sections);
        config_loop_timer.delay_micros(5000);
    }

    config_status_sendable.data.ready_for_config = 0;
    config_status_sendable.data.is_configured = 1;
    config_status_sendable.send_to_comms();
}

bool CommsLayer::initialize_hid() {
    // Initialize the HID physical layer
    m_hid.init();

    return true;
};

bool CommsLayer::initialize_ethernet() {
    // Initialize the Ethernet physical layer
    if (!m_ethernet.init()) {
        Serial.println("Ethernet initialization failed");
        return false;
    }
    
    Serial.println("Ethernet initialized successfully");

    return true;
};

}   // namespace Comms