#include "RefSystem.hpp"
#include "RefSystemPacketDefs.hpp"
#include "comms/data/sendable.hpp"

// Uncomment to enable debug prints
// #define REF_SYSTEM_DEBUG
// Uncomment to debug even harder
// #define REF_SYSTEM_DEBUG_ALL_FRAMES

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

RefSystem::RefSystem() { }

void RefSystem::init() {
    // clear and start the MCM serial
    MCM_SERIAL.clear();
    MCM_SERIAL.begin(112500);
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
    // return if over baud rate
    if (bytes_sent >= REF_MAX_BAUD_RATE) {
        Serial.println("Too many bytes");
        return;
    }

    // return if writing too many bytes
    if (length > REF_MAX_PACKET_SIZE) {
        Serial.println("Packet Too Long to Send!");
        return;
    }

    constexpr uint16_t frame_header_size = FrameHeader::packet_size;
    constexpr uint16_t command_ID_size = 2;
    constexpr uint16_t frame_tail_size = 2;
    constexpr uint16_t interaction_header_size = 6;
    constexpr uint16_t packet_overhead = frame_header_size + command_ID_size + frame_tail_size;

    if (length < frame_header_size) {
        Serial.println("Packet Too Short to Send!");
        return;
    }

    uint16_t frame_data_length = (static_cast<uint16_t>(packet[2]) << 8) | packet[1];
    if (frame_data_length > REF_MAX_DATA_SIZE) {
        Serial.println("Packet Data Too Long to Send!");
        return;
    }

    uint16_t expected_length = packet_overhead + frame_data_length;
    if (length != expected_length) {
        Serial.println("Packet Length Mismatch!");
        return;
    }

    if (frame_data_length < interaction_header_size) {
        Serial.println("Packet Data Too Short to Send!");
        return;
    }

    // update header
    packet[0] = 0xA5;                       // set SOF
    packet[3] = get_seq();                  // set SEQ
    packet[4] = generateCRC8(packet, 4);    // set CRC

    // update sender ID
    packet[9] = ref_data.robot_performance.robot_ID;
    packet[10] = ref_data.robot_performance.robot_ID >> 8;

    // update tail
    uint16_t footer_index = packet_overhead + frame_data_length - frame_tail_size;
    uint16_t footerCRC = generateCRC16(packet, footer_index);
    packet[footer_index] = (footerCRC & 0x00FF);    // set CRC
    packet[footer_index + 1] = (footerCRC >> 8);    // set CRC

    if (Serial7.write(packet, length) == length) {
        packets_sent++;
        bytes_sent += length;
    } else
        Serial.println("Failed to write");
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

    // increment buffer index
    buffer_index = bytes_read;

    return true;
}

bool RefSystem::read_frame_command_ID(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame) {
    // early return if serial is empty or not full enough
    if (serial->available() < 2)
        return false;

    // read and verify command ID
    int bytes_read = serial->readBytes(raw_buffer + buffer_index, 2);
    if (bytes_read != 2) {
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
    if (serial->available() < 2)
        return 0;

    // read and verify tail
    int bytes_read = serial->readBytes(raw_buffer + buffer_index, 2);
    if (bytes_read != 2) {
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

#if defined(REF_SYSTEM_DEBUG) && defined(REF_SYSTEM_DEBUG_ALL_FRAMES)
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
#ifdef REF_SYSTEM_DEBUG
        Serial.printf("[Ref][0x0101] reload_zone=%u capture_point=%u raw=0x%02x%02x%02x%02x\n",
                      ref_data.event_data.reload_zone_status, ref_data.event_data.capture_point_status,
                      frame.data[3], frame.data[2], frame.data[1], frame.data[0]);
#endif
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
        ref_data.robot_custom_controller_data.set_data(frame.data);
#ifdef REF_SYSTEM_DEBUG
        Serial.printf("[Ref][0x0309] custom-controller payload received (%u bytes)\n", frame.header.data_length);
#endif
        break;
    case FrameType::ROBOT_CUSTOM_CLIENT_DATA:
        ref_data.robot_custom_client_data.set_data(frame.data);
#ifdef REF_SYSTEM_DEBUG
        Serial.printf("[Ref][0x0310] custom-client payload received (%u bytes)\n", frame.header.data_length);
#endif
        break;
    case FrameType::CUSTOM_CLIENT_ROBOT_COMMAND: {
        ref_data.custom_client_robot_command.set_data(frame.data);
#ifdef REF_SYSTEM_DEBUG
        Serial.printf("[Ref][0x0311] custom-client command received (%u bytes)\n", frame.header.data_length);
#endif
        break;
    }
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
