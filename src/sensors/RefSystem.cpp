#include "RefSystem.hpp"
#include "RefSystemPacketDefs.hpp"
#include "comms/data/sendable.hpp"

// Uncomment to enable debug prints
// #define REF_SYSTEM_DEBUG

uint8_t generateCRC8(const uint8_t *data, uint32_t len) {
    uint8_t CRC8 = 0xFF;
    while (len-- > 0) {
        uint8_t curr = CRC8 ^ (*data++);
        CRC8 = CRC8Lookup[curr];
    }
    return CRC8;
}

uint16_t generateCRC16(const uint8_t *data, uint32_t len) {
    uint16_t CRC16 = 0xFFFF;
    while (len-- > 0) {
        uint8_t curr = *data++;
        CRC16 = (CRC16 >> 8) ^ CRC16Lookup[(CRC16 ^ static_cast<uint16_t>(curr)) & 0x00FF];
    }
    return CRC16;
}

static void put_u16(uint8_t* data, uint16_t value) {
    data[0] = value & 0x00FF;
    data[1] = value >> 8;
}

static uint16_t get_u16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

RefSystem::RefSystem() { }

void RefSystem::init() {
    // clear and start the MCM serial
    MCM_SERIAL.clear();
    MCM_SERIAL.begin(115200);
    MCM_SERIAL.flush();
    MCM_SERIAL.clear();

    // clear and start the VTM serial
    VTM_SERIAL.clear();
    VTM_SERIAL.begin(921600);
    VTM_SERIAL.flush();
    VTM_SERIAL.clear();
}

void RefSystem::read() {
    read_vtm();
    read_mcm();
}

void RefSystem::write(uint8_t* packet, uint16_t length) {
    write_frame(MCM_SERIAL, packet, length);
}

uint16_t RefSystem::get_client_id_for_robot(uint16_t robot_id) {
    switch (robot_id) {
    case 1:
        return 0x0101;
    case 2:
        return 0x0102;
    case 3:
        return 0x0103;
    case 4:
        return 0x0104;
    case 5:
        return 0x0105;
    case 6:
        return 0x0106;
    case 101:
        return 0x0165;
    case 102:
        return 0x0166;
    case 103:
        return 0x0167;
    case 104:
        return 0x0168;
    case 105:
        return 0x0169;
    case 106:
        return 0x016A;
    default:
        return 0;
    }
}

bool RefSystem::write_frame(HardwareSerial& serial, uint8_t* packet, uint16_t length) {
    if (packet == nullptr) {
        Serial.println("No Ref packet to send");
        return false;
    }

    if (length > REF_MAX_PACKET_SIZE) {
        Serial.println("Packet Too Long to Send!");
        return false;
    }

    if (length < REF_FRAME_OVERHEAD) {
        Serial.println("Packet Too Short to Send!");
        return false;
    }

    uint16_t frame_data_length = get_u16(packet + 1);
    if (frame_data_length > REF_MAX_DATA_SIZE) {
        Serial.println("Packet Data Too Long to Send!");
        return false;
    }

    uint16_t expected_length = REF_FRAME_OVERHEAD + frame_data_length;
    if (length != expected_length) {
        Serial.println("Packet Length Mismatch!");
        return false;
    }

    uint16_t command_id = get_u16(packet + FrameHeader::packet_size);
    if (command_id > REF_MAX_COMMAND_ID) {
        Serial.println("Invalid Ref command ID");
        return false;
    }

    uint32_t now_ms = millis();
    if (byte_window_start_ms == 0 || now_ms - byte_window_start_ms >= 1000) {
        byte_window_start_ms = now_ms;
        bytes_sent = 0;
    }

    if (bytes_sent + length > REF_MAX_BAUD_RATE) {
        Serial.println("Too many bytes");
        return false;
    }

    uint32_t now_us = micros();
    if (last_ref_packet_write_us != 0 && now_us - last_ref_packet_write_us < REF_MAX_PACKET_DELAY) {
        Serial.println("Ref packet rate limited");
        return false;
    }

    packet[0] = 0xA5;
    packet[3] = get_seq();
    packet[4] = generateCRC8(packet, 4);

    uint16_t footer_index = FrameHeader::packet_size + REF_COMMAND_ID_SIZE + frame_data_length;
    uint16_t footerCRC = generateCRC16(packet, footer_index);
    packet[footer_index] = footerCRC & 0x00FF;
    packet[footer_index + 1] = footerCRC >> 8;

    if (serial.write(packet, length) == length) {
        packets_sent++;
        bytes_sent += length;
        last_ref_packet_write_us = now_us;
        return true;
    }

    Serial.println("Failed to write");
    return false;
}

bool RefSystem::write_robot_interaction(uint16_t content_id, const uint8_t* payload, uint16_t payload_length, uint16_t receiver_id) {
    if (payload_length > RobotInteraction::max_content_size) {
        Serial.println("Robot interaction payload too long");
        return false;
    }

    if (payload_length > 0 && payload == nullptr) {
        Serial.println("No robot interaction payload");
        return false;
    }

    uint16_t sender_id = ref_data.robot_performance.robot_ID;
    if (sender_id == 0) {
        Serial.println("No robot ID for client drawing");
        return false;
    }

    if (receiver_id == 0) {
        receiver_id = get_client_id_for_robot(sender_id);
    }

    if (receiver_id == 0) {
        Serial.println("No player client ID for this robot");
        return false;
    }

    uint16_t frame_data_length = RobotInteraction::header_size + payload_length;
    uint16_t packet_length = REF_FRAME_OVERHEAD + frame_data_length;
    uint8_t packet[REF_MAX_PACKET_SIZE] = {0};

    put_u16(packet + 1, frame_data_length);
    put_u16(packet + FrameHeader::packet_size, static_cast<uint16_t>(FrameType::ROBOT_INTERACTION));
    put_u16(packet + 7, content_id);
    put_u16(packet + 9, sender_id);
    put_u16(packet + 11, receiver_id);

    if (payload_length > 0) {
        memcpy(packet + 7 + RobotInteraction::header_size, payload, payload_length);
    }

    return write_frame(MCM_SERIAL, packet, packet_length);
}

void RefSystem::send_to_comms() {
    CommsRefData ref_data_for_comms;
    ref_data_for_comms.game_status_data = ref_data.game_status.to_comms_data();
    ref_data_for_comms.game_result_data = ref_data.game_result.to_comms_data();
    ref_data_for_comms.robot_health_data = ref_data.game_robot_hp.to_comms_data();
    ref_data_for_comms.game_event_data = ref_data.event_data.to_comms_data();
    ref_data_for_comms.robot_performance_data = ref_data.robot_performance.to_comms_data();
    ref_data_for_comms.robot_power_heat_data = ref_data.robot_power_heat.to_comms_data();
    ref_data_for_comms.damage_status_data = ref_data.damage_status.to_comms_data();
    ref_data_for_comms.launching_status_data = ref_data.launching_status.to_comms_data();
    ref_data_for_comms.projectile_allowance_data = ref_data.projectile_allowance.to_comms_data();
    Comms::Sendable<CommsRefData> ref_data_sendable = ref_data_for_comms;
    ref_data_sendable.send_to_comms();
}

bool RefSystem::read_frame_header(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame) {
    // early return if serial is empty or not full enough
    if (serial->available() < FrameHeader::packet_size)
        return false;

    // verify that what we are about to read is a header (first byte must be 0xA5)
    while (serial->peek() != 0xA5 && serial->peek() != -1) {
        serial->read();
    }

    // early return if serial is empty or not full enough
    if (serial->available() < FrameHeader::packet_size)
        return false;

    // read and verify header
    int bytes_read = serial->readBytes(raw_buffer, FrameHeader::packet_size);
    if (bytes_read != FrameHeader::packet_size) {
        Serial.println("Couldnt read enough bytes for Header");
        packets_failed++;
        return false;
    }

    // set read data
    frame.header.SOF = raw_buffer[buffer_index + 0];
    if (frame.header.SOF != 0xA5) {
        Serial.println("Not a valid frame");
        return false;
    }

    frame.header.data_length = (raw_buffer[buffer_index + 2] << 8) | raw_buffer[buffer_index + 1];
    frame.header.sequence = raw_buffer[buffer_index + 3];
    frame.header.CRC = raw_buffer[buffer_index + 4];

    // verify the CRC is correct
    uint8_t expected_CRC = generateCRC8(raw_buffer, 4);
    if (frame.header.CRC != expected_CRC) {
        Serial.printf("[Ref] Header CRC failed: received=0x%02x expected=0x%02x\n", frame.header.CRC, expected_CRC);
        packets_failed++;
        return false;
    }

    if (frame.header.data_length > REF_MAX_DATA_SIZE) {
        Serial.printf("[Ref] Data length too large: %u (max %u)\n", frame.header.data_length, REF_MAX_DATA_SIZE);
        packets_failed++;
        return false;
    }

    // increment buffer index
    buffer_index = bytes_read;

    return true;
}

bool RefSystem::read_frame_command_ID(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame) {
    // early return if serial is empty or not full enough
    if (serial->available() < REF_COMMAND_ID_SIZE)
        return false;

    // read and verify command ID
    int bytes_read = serial->readBytes(raw_buffer + buffer_index, REF_COMMAND_ID_SIZE);
    if (bytes_read != REF_COMMAND_ID_SIZE) {
        Serial.println("Couldnt read enough bytes for ID");
        packets_failed++;
        return false;
    }

    // assign read ID
    frame.commandID = (raw_buffer[buffer_index + 1] << 8) | raw_buffer[buffer_index + 0];

    // sanity check, verify the ID is valid
    if (frame.commandID > REF_MAX_COMMAND_ID) {
        Serial.printf("[Ref] Invalid command ID: 0x%04x\n", frame.commandID);
        packets_failed++;
        return false;
    }

    // increment buffer index
    buffer_index += bytes_read;

    return true;
}

bool RefSystem::read_frame_data(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame) {
    // early return if serial is empty or not full enough
    if (serial->available() < frame.header.data_length)
        return false;

    // read and verify data
    int bytes_read = serial->readBytes(raw_buffer + buffer_index, frame.header.data_length);
    if (bytes_read != frame.header.data_length) {
        Serial.println("Couldnt read enough bytes for Data");
        packets_failed++;
        return false;
    }

    // set read data
    memcpy(frame.data.data, raw_buffer + buffer_index, bytes_read);

    // increment buffer index
    buffer_index += bytes_read;

    return true;
}

int RefSystem::read_frame_tail(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame) {
    // early return if serial is empty or not full enough
    if (serial->available() < REF_FRAME_TAIL_SIZE)
        return 0;

    // read and verify tail
    int bytes_read = serial->readBytes(raw_buffer + buffer_index, REF_FRAME_TAIL_SIZE);
    if (bytes_read != REF_FRAME_TAIL_SIZE) {
        Serial.println("Couldnt read enough bytes for CRC");
        packets_failed++;
        return 0;
    }

    // store CRC
    frame.CRC = (raw_buffer[buffer_index + 1] << 8) | raw_buffer[buffer_index + 0];

    uint16_t expected_CRC = generateCRC16(raw_buffer, buffer_index);
    if (frame.CRC != expected_CRC) {
        Serial.printf("[Ref] Tail CRC failed: command=0x%04x length=%u received=0x%04x expected=0x%04x\n",
                      frame.commandID, frame.header.data_length, frame.CRC, expected_CRC);
        packets_failed++;
        return -1;
    }


    // increment buffer index
    buffer_index = bytes_read;

    return 1;
}

void RefSystem::set_ref_data(Frame& frame, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2]) {
    // Copy the header
    frame.header.SOF = raw_buffer[0];
    frame.header.data_length = (raw_buffer[2] << 8) | raw_buffer[1];
    frame.header.sequence = raw_buffer[3];
    frame.header.CRC = raw_buffer[4];
    // copy the command ID
    frame.commandID = (raw_buffer[6] << 8) | raw_buffer[5];
    // copy the data
    memcpy(frame.data.data, raw_buffer + 7, frame.header.data_length);
    // copy the CRC
    frame.CRC = (raw_buffer[7 + frame.header.data_length] << 8) | raw_buffer[6 + frame.header.data_length];

    // grab the type
    FrameType type = static_cast<FrameType>(frame.commandID);

#ifdef REF_SYSTEM_DEBUG
    Serial.printf("[Ref] command=0x%04x length=%u sequence=%u\n",
                  frame.commandID, frame.header.data_length, frame.header.sequence);
#endif

    switch (type) {
    case FrameType::GAME_STATUS:
        ref_data.game_status.set_data(frame.data);
        break;
    case FrameType::GAME_RESULT:
        ref_data.game_result.set_data(frame.data);
        break;
    case FrameType::GAME_ROBOT_HP:
        ref_data.game_robot_hp.set_data(frame.data);
        break;
    case FrameType::EVENT_DATA:
        ref_data.event_data.set_data(frame.data);
        break;
    case FrameType::REFEREE_WARNING:
        ref_data.referee_warning.set_data(frame.data);
        break;
    case FrameType::DART_STATUS:
        ref_data.dart_status.set_data(frame.data);
        break;
    case FrameType::ROBOT_PERFORMANCE:
        ref_data.robot_performance.set_data(frame.data);
        break;
    case FrameType::ROBOT_POWER_HEAT:
        ref_data.robot_power_heat.set_data(frame.data);
        break;
    case FrameType::ROBOT_POSITION:
        ref_data.robot_position.set_data(frame.data);
        break;
    case FrameType::ROBOT_BUFF:
        ref_data.robot_buff.set_data(frame.data);
        break;
    case FrameType::DAMAGE_STATUS:
        ref_data.damage_status.set_data(frame.data);
        damage_status_changed = true;
        break;
    case FrameType::LAUNCHING_STATUS:
        ref_data.launching_status.set_data(frame.data);
        break;
    case FrameType::PROJECTILE_ALLOWANCE:
        ref_data.projectile_allowance.set_data(frame.data);
        break;
    case FrameType::RFID_STATUS:
        ref_data.rfid_status.set_data(frame.data);
        break;
    case FrameType::DART_COMMAND:
        ref_data.dart_command.set_data(frame.data);
        break;
    case FrameType::GROUND_ROBOT_POSITIONS:
        ref_data.ground_robot_positions.set_data(frame.data);
        break;
    case FrameType::RADAR_PROGRESS:
        ref_data.radar_progress.set_data(frame.data);
        break;
    case FrameType::SENTRY_DECISION:
        ref_data.sentry_decision.set_data(frame.data);
        break;
    case FrameType::RADAR_DECISION:
        ref_data.radar_decision.set_data(frame.data);
        break;
    case FrameType::ROBOT_INTERACTION:
        // ref_data.robot_interaction.set_data(frame.data);
        // todo: implement before china
        break;
    case FrameType::CUSTOM_CONTROLLER_ROBOT:
        ref_data.custom_controller_robot.set_data(frame.data);
        break;
    case FrameType::SMALL_MAP_COMMAND:
        ref_data.small_map_command.set_data(frame.data);
        break;
    case FrameType::SMALL_MAP_RADAR_POSITION:
        ref_data.small_map_radar_position.set_data(frame.data);
        break;
    case FrameType::CUSTOM_CONTROLLER_CLIENT:
        ref_data.custom_controller_client.set_data(frame.data);
        break;
    case FrameType::SMALL_MAP_SENTRY_COMMAND:
        ref_data.small_map_sentry_command.set_data(frame.data);
        break;
    case FrameType::SMALL_MAP_ROBOT_DATA:
        ref_data.small_map_robot_data.set_data(frame.data);
        break;
    case FrameType::ROBOT_CUSTOM_CONTROLLER_DATA:
        break;
    case FrameType::ROBOT_CUSTOM_CLIENT_DATA:
        break;
    case FrameType::CUSTOM_CLIENT_ROBOT_COMMAND:
        break;
    default:
        Serial.printf("Ref System::set_ref_data: Unknown Frame Type 0x%04x\n", frame.commandID);
        break;
    }
}

void RefSystem::read_vtm() {
    while (VTM_SERIAL.available() >= VTM_REMOTE_CONTROL_PACKET_SIZE) {
        while (VTM_SERIAL.available() >= VTM_REMOTE_CONTROL_PACKET_SIZE && VTM_SERIAL.peek() != VTM_REMOTE_CONTROL_HEADER_1) {
            VTM_SERIAL.read();
        }

        if (VTM_SERIAL.available() < VTM_REMOTE_CONTROL_PACKET_SIZE) {
            return;
        }

        uint8_t packet[VTM_REMOTE_CONTROL_PACKET_SIZE] = {0};
        int bytes_read = VTM_SERIAL.readBytes(packet, VTM_REMOTE_CONTROL_PACKET_SIZE);
        if (bytes_read != VTM_REMOTE_CONTROL_PACKET_SIZE) {
            return;
        }

        if (packet[1] != VTM_REMOTE_CONTROL_HEADER_2) {
            packets_failed++;
            continue;
        }

        uint16_t received_crc = packet[19] | (static_cast<uint16_t>(packet[20]) << 8);
        uint16_t expected_crc = generateCRC16(packet, VTM_REMOTE_CONTROL_PACKET_SIZE - 2);
        if (received_crc != expected_crc) {
            Serial.printf("[VTM] CRC failed: received=0x%04x expected=0x%04x\n", received_crc, expected_crc);
            packets_failed++;
            continue;
        }

        ref_data.vtm_remote_control.set_data(packet);
        packets_received++;

#ifdef REF_SYSTEM_DEBUG
        const VTMRemoteControl &input = ref_data.vtm_remote_control;
        Serial.printf("[VTM] mouse=(%d,%d) scroll=%d buttons=%u/%u/%u keys=0x%04x\n", input.mouse_speed_x, input.mouse_speed_y, input.scroll_speed, static_cast<uint32_t>(input.button_left), static_cast<uint32_t>(input.button_right), static_cast<uint32_t>(input.button_middle), input.keyboard_value);
#endif
        return;
    }
}

void RefSystem::read_mcm() {
    bool success = true;

    // try to read the header
    if (success && !mcm_data.header_read) {
        success = read_frame_header(&MCM_SERIAL, mcm_data.raw_buffer, mcm_data.buffer_index, mcm_data.curr_frame);
        mcm_data.header_read = success;
    }

    // try to read the ID
    if (success && !mcm_data.command_ID_read) {
        success = read_frame_command_ID(&MCM_SERIAL, mcm_data.raw_buffer, mcm_data.buffer_index, mcm_data.curr_frame);
        mcm_data.command_ID_read = success;
    }

    // try to read the data
    if (success && !mcm_data.data_read) {
        success = read_frame_data(&MCM_SERIAL, mcm_data.raw_buffer, mcm_data.buffer_index, mcm_data.curr_frame);
        mcm_data.data_read = success;
    }

    // try to read the tail
    if (success && !mcm_data.tail_read) {
        int tail_ret = read_frame_tail(&MCM_SERIAL, mcm_data.raw_buffer, mcm_data.buffer_index, mcm_data.curr_frame);

        if (tail_ret == 1) {
            success = true;
            mcm_data.tail_read = true;
        } else if (tail_ret == -1) {
            success = false;

            // crc failed so reset the frame
            mcm_data.header_read = false;
            mcm_data.command_ID_read = false;
            mcm_data.data_read = false;
            mcm_data.tail_read = false;
            mcm_data.buffer_index = 0;
            memset(mcm_data.raw_buffer, 0, REF_MAX_PACKET_SIZE * 2);
        } else {
            success = false;
        }
    }

    // process the data
    if (success) {
        set_ref_data(mcm_data.curr_frame, mcm_data.raw_buffer);

        // reset flags
        mcm_data.header_read = false;
        mcm_data.command_ID_read = false;
        mcm_data.data_read = false;
        mcm_data.tail_read = false;
        mcm_data.buffer_index = 0;
        memset(mcm_data.raw_buffer, 0, REF_MAX_PACKET_SIZE * 2);
    }
}
