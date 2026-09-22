#include "comms_layer.hpp"
#include "utils/system_log.hpp"
#include "comms/data/configuration_status_data.hpp"
#include "comms/data/sendable.hpp"
#include "utils/safety.hpp"


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

CommsLayer comms_layer;
	
CommsLayer::CommsLayer() {
    SystemLog.info(Subsystem::COMMS,"CommsLayer: constructed\n");
};

CommsLayer::~CommsLayer() {
    SystemLog.info(Subsystem::COMMS,"CommsLayer: destructed\n");
};

int CommsLayer::init() {
    SystemLog.info(Subsystem::COMMS,"CommsLayer: initializing\n");
    
    // hid failing is a fatal error
    bool hid_init = initialize_hid();
    if (!hid_init) {
        SystemLog.error(Subsystem::COMMS,"CommsLayer: HIDComms init failed\n");
        return -1;
    }

    // ethernet init failing is not a fatal error
    bool ethernet_init = initialize_ethernet();
    if (!ethernet_init) {
        SystemLog.info(Subsystem::COMMS,"CommsLayer: EthernetComms init failed\n");
    }

    SystemLog.info(Subsystem::COMMS,"CommsLayer: initialized\n");

    return 0;
};

int CommsLayer::run() {
    // read packets from the physical layers
    receive_packets();

    // write packets to the physical layers
    send_packets();

    return 0;
};

void CommsLayer::queue_data(const CommsData* data) {
    if (data == nullptr) {
        safety::assert_or_safety_procedure(false, "CommsLayer::queue_data: Data is null");
        return;
    }
    switch (data->physical_medium) {
    case PhysicalMedium::HID:
        if (!is_hid_connected()) {
            // discard attempt to send
            SystemLog.warn(Subsystem::COMMS,"Attempting to re-route %s to HID but HID is not connected\n", to_string(data->type_label).c_str());
            break;
        }
        (void)m_hid_payload.add(data);
        break;
    case PhysicalMedium::Ethernet:
        // if ethernet is down and it is a small enough packet, route it through HID instead
        if (!is_ethernet_connected() && data->size < HID_PACKET_PAYLOAD_SIZE) {
            (void)m_hid_payload.add(data);
            break;
        } else if (data->size > HID_PACKET_PAYLOAD_SIZE) {
            // discard attempt to send
            SystemLog.warn(Subsystem::COMMS,"Attempting to re-route %s to HID but packet is too large\n", to_string(data->type_label).c_str());
            break;
        }

        (void)m_ethernet_payload.add(data);
        break;
    default:
        assert(false && "Invalid PhysicalMedium");
    }
};

void CommsLayer::send_packets() {
    // HID always transmits its full fixed-size payload, so clear bytes beyond
    // the complete records and sentinel written during construction.
    memset(m_hid_outgoing.payload(), 0, HID_PACKET_PAYLOAD_SIZE);
    (void)m_hid_payload.construct_data(m_hid_outgoing.payload());
    m_hid.send_packet(m_hid_outgoing);

    // Ethernet transmits only the complete record bytes returned by construction.
    const uint16_t payload_size = m_ethernet_payload.construct_data(m_ethernet_outgoing.payload());
    m_last_ethernet_send_payload_size = payload_size;
    const uint32_t packet_size = PACKET_HEADER_SIZE + payload_size;
    m_ethernet.send_packet(m_ethernet_outgoing, packet_size);
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
    Sendable<ConfigurationStatusData> config_status_sendable;
    while (!m_hive_data.config.config_start.num_config_sections != 0) {
        SystemLog.info(Subsystem::COMMS,"Waiting for config start packet... time since start: %d ms\n", millis() - time);
        config_status_sendable.data.is_configured = 0;
        config_status_sendable.send_to_comms();
        run();
        config_loop_timer.delay_micros(5000);
    }
    SystemLog.info(Subsystem::COMMS,"Config start packet received, expecting %d config sections\n", m_hive_data.config.config_start.num_config_sections);

    while(!m_hive_data.config.is_configured()) {
        config_status_sendable.data.ready_for_config = 1;
        config_status_sendable.send_to_comms();
        run();
        SystemLog.info(Subsystem::COMMS,"Config: received %d of %d sections\n", m_hive_data.config.num_sections_received, m_hive_data.config.config_start.num_config_sections);
        config_loop_timer.delay_micros(5000);
    }
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

void CommsLayer::print_live_data() {
    bool eth_init = m_ethernet.is_initialized();
    bool eth_conn = m_ethernet.is_connected();
    bool eth_link = m_ethernet.is_link_up();
    bool hid_init = m_hid.is_initialized();
    bool hid_conn = m_hid.is_connected();
    bool configured = is_configured();

    // Statistics and rate calculations
    static uint32_t last_calc_time = 0;
    static uint32_t last_tx_pkts = 0;
    static uint32_t last_rx_pkts = 0;
    static uint32_t last_tx_bytes = 0;
    static uint32_t last_rx_bytes = 0;
    static float tx_rate_hz = 0.0f;
    static float rx_rate_hz = 0.0f;
    static float tx_rate_kbs = 0.0f;
    static float rx_rate_kbs = 0.0f;

    uint32_t now = millis();
    uint32_t dt_ms = now - last_calc_time;
    uint32_t tx_pkts = m_ethernet.get_packets_sent();
    uint32_t rx_pkts = m_ethernet.get_packets_received();
    uint32_t tx_bytes = m_ethernet.get_total_bytes_sent();
    uint32_t rx_bytes = m_ethernet.get_total_bytes_received();

    if (dt_ms >= 500) {
        if (last_calc_time != 0 && dt_ms > 0) {
            tx_rate_hz = (float)(tx_pkts - last_tx_pkts) * 1000.0f / (float)dt_ms;
            rx_rate_hz = (float)(rx_pkts - last_rx_pkts) * 1000.0f / (float)dt_ms;
            tx_rate_kbs = (float)(tx_bytes - last_tx_bytes) * 1000.0f / (1024.0f * (float)dt_ms);
            rx_rate_kbs = (float)(rx_bytes - last_rx_bytes) * 1000.0f / (1024.0f * (float)dt_ms);
        }
        last_calc_time = now;
        last_tx_pkts = tx_pkts;
        last_rx_pkts = rx_pkts;
        last_tx_bytes = tx_bytes;
        last_rx_bytes = rx_bytes;
    }

    uint32_t now_us = micros();
    uint32_t time_since_tx_us = m_ethernet.get_last_send_time() > 0 ? (now_us - m_ethernet.get_last_send_time()) : 0;
    uint32_t time_since_rx_us = m_ethernet.get_last_recv_time() > 0 ? (now_us - m_ethernet.get_last_recv_time()) : 0;

    Serial.printf("=== LIVE COMMS STATUS ===\033[K\n");
    Serial.printf(" Status        : Eth: %s (%s) | HID: %s | Configured: %s\033[K\n",
                  eth_init ? (eth_conn ? "CONNECTED" : "DISCONNECTED") : "OFFLINE",
                  eth_link ? "LINK UP" : "NO LINK",
                  hid_init ? (hid_conn ? "CONNECTED" : "DISCONNECTED") : "OFFLINE",
                  configured ? "YES" : "NO");
    Serial.printf(" Network       : Jetson %u.%u.%u.%u:%u <-> Teensy %u.%u.%u.%u:%u\033[K\n",
                  m_ethernet.get_jetson_ip()[0], m_ethernet.get_jetson_ip()[1],
                  m_ethernet.get_jetson_ip()[2], m_ethernet.get_jetson_ip()[3],
                  m_ethernet.get_jetson_port(),
                  m_ethernet.get_teensy_ip()[0], m_ethernet.get_teensy_ip()[1],
                  m_ethernet.get_teensy_ip()[2], m_ethernet.get_teensy_ip()[3],
                  m_ethernet.get_teensy_port());
    Serial.printf("----------------------------------------------------------------------\033[K\n");
    Serial.printf(" ETHERNET TX (Teensy -> Jetson)\033[K\n");
    Serial.printf("   Packets     : Sent: %lu | Failed: %lu | Rate: %6.1f Hz (%5.1f KB/s)\033[K\n",
                  (unsigned long)tx_pkts,
                  (unsigned long)m_ethernet.get_packets_send_failed(),
                  tx_rate_hz, tx_rate_kbs);
    uint32_t last_tx_pkt_sz = m_ethernet.get_last_send_packet_size();
    uint16_t last_tx_pld_sz = m_last_ethernet_send_payload_size;
    float pld_pct = (float)last_tx_pld_sz * 100.0f / (float)ETHERNET_PACKET_PAYLOAD_SIZE;
    Serial.printf("   Packet Size : %lu B [Hdr: %lu B | Payload: %u / %lu B (%.1f%%)]\033[K\n",
                  (unsigned long)last_tx_pkt_sz,
                  (unsigned long)PACKET_HEADER_SIZE,
                  (unsigned int)last_tx_pld_sz,
                  (unsigned long)ETHERNET_PACKET_PAYLOAD_SIZE,
                  pld_pct);
    Serial.printf("   Staged      : High: %u | Medium: %u | Dropped: %lu\033[K\n",
                  (unsigned int)m_ethernet_payload.get_high_priority_queue_size(),
                  (unsigned int)m_ethernet_payload.get_medium_priority_queue_size(),
                  (unsigned long)m_ethernet_payload.get_dropped_record_count());
    if (m_ethernet.get_last_send_time() > 0) {
        Serial.printf("   Last TX     : %.2f ms ago\033[K\n", (float)time_since_tx_us / 1000.0f);
    } else {
        Serial.printf("   Last TX     : Never\033[K\n");
    }
    Serial.printf("----------------------------------------------------------------------\033[K\n");
    Serial.printf(" ETHERNET RX (Jetson -> Teensy)\033[K\n");
    Serial.printf("   Packets     : Recv: %lu | Dropped: %lu | Rate: %6.1f Hz (%5.1f KB/s)\033[K\n",
                  (unsigned long)rx_pkts,
                  (unsigned long)m_ethernet.get_packets_recv_failed(),
                  rx_rate_hz, rx_rate_kbs);
    uint32_t last_rx_pkt_sz = m_ethernet.get_last_recv_packet_size();
    int32_t last_rx_err_sz = m_ethernet.get_last_recv_error_size();
    if (last_rx_pkt_sz > 0) {
        Serial.printf("   Last Valid  : %lu B [Hdr: %lu B | Payload: %lu B]\033[K\n",
                      (unsigned long)last_rx_pkt_sz,
                      (unsigned long)PACKET_HEADER_SIZE,
                      (unsigned long)(last_rx_pkt_sz >= PACKET_HEADER_SIZE ? last_rx_pkt_sz - PACKET_HEADER_SIZE : 0));
    } else {
        Serial.printf("   Last Valid  : None\033[K\n");
    }
    if (last_rx_err_sz != 0) {
        Serial.printf("   Last Error  : %ld B (Expected: %lu B [FIXED_MAX])\033[K\n",
                      (long)last_rx_err_sz, (unsigned long)ETHERNET_PACKET_MAX_SIZE);
    } else {
        Serial.printf("   Last Error  : None\033[K\n");
    }
    if (m_ethernet.get_last_recv_time() > 0) {
        Serial.printf("   Last RX     : %.2f ms ago (Timeout: %lu ms)\033[K\n",
                      (float)time_since_rx_us / 1000.0f,
                      (unsigned long)(m_ethernet.get_connection_timeout() / 1000));
    } else {
        Serial.printf("   Last RX     : Never (Timeout: %lu ms)\033[K\n",
                      (unsigned long)(m_ethernet.get_connection_timeout() / 1000));
    }
    Serial.printf("----------------------------------------------------------------------\033[K\n");
    Serial.printf(" HID STATS     : Read: %llu | Sent: %llu | Failed: %llu\033[K\n",
                  (unsigned long long)m_hid.get_packets_read(),
                  (unsigned long long)m_hid.get_packets_sent(),
                  (unsigned long long)m_hid.get_packets_failed());
    Serial.printf("   Staged      : High: %u | Medium: %u | Dropped: %lu\033[K\n",
                  (unsigned int)m_hid_payload.get_high_priority_queue_size(),
                  (unsigned int)m_hid_payload.get_medium_priority_queue_size(),
                  (unsigned long)m_hid_payload.get_dropped_record_count());
}

}   // namespace Comms
