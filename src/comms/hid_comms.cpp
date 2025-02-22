#include "hid_comms.hpp"

namespace Comms {

uint8_t HIDPacket::get_id() {
    // c++ moment
    char* x = raw;
    return *reinterpret_cast<uint8_t*>(x + TEENSY_PACKET_ID_OFFSET);
}

uint8_t HIDPacket::get_info() {
    return *reinterpret_cast<uint8_t*>(raw + TEENSY_PACKET_INFO_OFFSET);
}

void HIDPacket::set_id(uint16_t id) {
    // c++ moment
    char* x = raw;
    *reinterpret_cast<uint16_t*>(x + TEENSY_PACKET_ID_OFFSET) = id;
}

void HIDPacket::set_info(uint8_t info) {
    *reinterpret_cast<uint8_t*>(raw + TEENSY_PACKET_INFO_OFFSET) = info;
}

void HIDPacket::get_target_state(float state[STATE_LEN][3]) {
    memcpy(state, raw + KHADAS_PACKET_TSTATE_OFFSET, sizeof(float) * STATE_LEN * 3);
}

uint8_t HIDPacket::get_hive_override_request() {
    return *reinterpret_cast<uint8_t*>(raw + KHADAS_PACKET_HIVE_OVERRIDE_STATE_REQUEST_OFFSET);
}

void HIDPacket::get_hive_override_state(float state[STATE_LEN][3]) {
    memcpy(state, raw + KHADAS_PACKET_HIVE_OVERRIDE_STATE_OFFSET, sizeof(float) * STATE_LEN * 3);
}

void HIDPacket::get_ref_draw_data(char** draw_data) {}

void HIDPacket::set_time(double time) {
    memcpy(raw + TEENSY_PACKET_TIME_OFFSET, &time, sizeof(double));
}

void HIDPacket::set_estimated_state(float state[STATE_LEN][3]) {
    memcpy(raw + TEENSY_PACKET_ESTATE_OFFSET, state, sizeof(float) * STATE_LEN * 3);
}

void HIDPacket::set_sensor_data(SensorData* sensor_data) {
    memcpy(raw + TEENSY_PACKET_SENSOR_OFFSET, sensor_data->raw, sizeof(SensorData));
}

void HIDPacket::set_ref_data(uint8_t ref_data[180]) {
    memcpy(raw + TEENSY_PACKET_REF_OFFSET, ref_data, 180);
}

HIDComms::HIDComms() {}

void HIDComms::init() { Serial.println("Starting HID layer"); }

std::optional<HIDPacket> HIDComms::sendReceive(HIDPacket& outgoing_packet) {
    // loop until the packet buffer is empty (this is only ever 1 packet large)
    while (usb_rawhid_available()) {
        HIDPacket incoming_packet;
        
        // attempt to read
        if (read(incoming_packet)) {
            // if we read, attempt to write
            if (!write(outgoing_packet))
                Serial.printf("Failed to send: %llu\n", m_packetsSent);

            return m_incomingPacket;
        }
    }

    return {};
}

void HIDComms::print_outgoing() {
    Serial.println("Outgoing packet:");
    for (unsigned int i = 0; i < HID_PACKET_SIZE; i++)
        Serial.printf("%.2x ", m_outgoingPacket.raw[i]);

    Serial.println();
}

void HIDComms::print_incoming() {
    Serial.println("Incoming packet:");
    for (unsigned int i = 0; i < HID_PACKET_SIZE; i++)
        Serial.printf("%.2x ", m_incomingPacket.raw[i]);

    Serial.println();
}

bool HIDComms::read(HIDPacket& incoming_packet) {
    // attempt to read a full packet
    // this has no timeout
    int bytes_read = usb_rawhid_recv(incoming_packet.raw, 0);
    if (bytes_read == HID_PACKET_SIZE) {
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
    outgoing_packet.raw[0] = 0xff;

    // attempt to write a full packet
    // this has no timeout
    int bytes_sent = usb_rawhid_send(outgoing_packet.raw, 0);
    if (bytes_sent == HID_PACKET_SIZE) {
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