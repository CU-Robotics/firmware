#include "usb_hid.hpp"

uint8_t CommsPacket::get_id() {
    // c++ moment
    char* x = raw;
    return *reinterpret_cast<uint8_t*>(x + TEENSY_PACKET_ID_OFFSET);
}

uint8_t CommsPacket::get_info() {
    return *reinterpret_cast<uint8_t*>(raw + TEENSY_PACKET_INFO_OFFSET);
}

void CommsPacket::set_id(uint16_t id) {
    // c++ moment
    char* x = raw;
    *reinterpret_cast<uint16_t*>(x + TEENSY_PACKET_ID_OFFSET) = id;
}

void CommsPacket::set_info(uint8_t info) {
    *reinterpret_cast<uint8_t*>(raw + TEENSY_PACKET_INFO_OFFSET) = info;
}

void CommsPacket::get_target_state(float state[STATE_LEN][3]) {
    memcpy(state, raw + KHADAS_PACKET_TSTATE_OFFSET, sizeof(float) * STATE_LEN * 3);
}

uint8_t CommsPacket::get_hive_override_request() {
    return *reinterpret_cast<uint8_t*>(raw + KHADAS_PACKET_HIVE_OVERRIDE_STATE_REQUEST_OFFSET);
}

void CommsPacket::get_hive_override_state(float state[STATE_LEN][3]) {
    memcpy(state, raw + KHADAS_PACKET_HIVE_OVERRIDE_STATE_OFFSET, sizeof(float) * STATE_LEN * 3);
}

void CommsPacket::get_ref_draw_data(char** draw_data) {}

void CommsPacket::set_time(double time) {
    memcpy(raw + TEENSY_PACKET_TIME_OFFSET, &time, sizeof(double));
}

void CommsPacket::set_estimated_state(float state[STATE_LEN][3]) {
    memcpy(raw + TEENSY_PACKET_ESTATE_OFFSET, state, sizeof(float) * STATE_LEN * 3);
}

void CommsPacket::set_sensor_data(SensorData* sensor_data) {
    memcpy(raw + TEENSY_PACKET_SENSOR_OFFSET, sensor_data->raw, sizeof(SensorData));
}

void CommsPacket::set_ref_data(uint8_t ref_data[180]) {
    memcpy(raw + TEENSY_PACKET_REF_OFFSET, ref_data, 180);
}

HIDLayer::HIDLayer() {}

void HIDLayer::init() { Serial.println("Starting HID layer"); }

void HIDLayer::ping() {
    // loop until the packet buffer is empty (this is only ever 1 packet large)
    while (usb_rawhid_available()) {
        // attempt to read
        if (read()) {
            // if we read, attempt to write
            if (!write())
                Serial.printf("Failed to send ping %llu\n", m_packetsSent);
        }
    }
}

void HIDLayer::print_outgoing() {
    Serial.println("Outgoing packet:");
    for (unsigned int i = 0; i < COMMS_PACKET_SIZE; i++)
        Serial.printf("%.2x ", m_outgoingPacket.raw[i]);

    Serial.println();
}

void HIDLayer::print_incoming() {
    Serial.println("Incoming packet:");
    for (unsigned int i = 0; i < COMMS_PACKET_SIZE; i++)
        Serial.printf("%.2x ", m_incomingPacket.raw[i]);

    Serial.println();
}

bool HIDLayer::read() {
    // attempt to read a full packet
    // this has no timeout
    int bytes_read = usb_rawhid_recv(m_incomingPacket.raw, 0);
    if (bytes_read == COMMS_PACKET_SIZE) {
        // increment total number of packets read and return success
        m_packetsRead++;
        return true;
    } else {
        return false;
    }
}

bool HIDLayer::write() {
    // verify that the first byte is set to something (0xff)
    // prevents a weird comms issue where the whole packet is shifted left by one byte if the first byte is not ever set
    m_outgoingPacket.raw[0] = 0xff;

    // attempt to write a full packet
    // this has no timeout
    int bytes_sent = usb_rawhid_send(m_outgoingPacket.raw, 0);
    if (bytes_sent == COMMS_PACKET_SIZE) {
        // increment total number of packets sent and return success
        m_packetsSent++;
        return true;
    } else {
        Serial.println("Comms: failed write");
        m_packetsFailed++;
        return false;
    }
}