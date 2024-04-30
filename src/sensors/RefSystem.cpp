#include "RefSystem.hpp"

uint8_t generateCRC8(uint8_t* data, uint32_t len) {
    uint8_t CRC8 = 0xFF;
    while (len-- > 0) {
        uint8_t curr = CRC8 ^ (*data++);
        CRC8 = CRC8Lookup[curr];
    }
    return CRC8;
}

uint16_t generateCRC16(uint8_t* data, uint32_t len) {
    uint16_t CRC16 = 0xFFFF;
    while (len-- > 0) {
        uint8_t curr = *data++;
        CRC16 = (CRC16 >> 8) ^ CRC16Lookup[(CRC16 ^ static_cast<uint16_t>(curr)) & 0x00FF];
    }
    return CRC16;
}

RefSystem::RefSystem() {}

void RefSystem::init() {
    // clear and start Serial2
    Serial2.clear();
    Serial2.begin(112500);
    Serial2.flush();
    Serial2.clear();
}

void RefSystem::read() {
    // Dont read if there is not enough data for a full packet
    // This prevents ref from just reading sections of headers, but not the rest of the packet
    // this is a stop-gap solution, should be re-written to be more robust
    if (Serial2.available() < 255)
        return;

    Frame frame{};

    bool success = true;

    // reset buffer
    memset(raw_buffer, 0, REF_MAX_PACKET_SIZE);

    // reset buffer index
    buffer_index = 0;

    // read header
    if (success)
        success = read_frame_header(frame);

    // read ID
    if (success)
        success = read_frame_command_ID(frame);

    // read data
    if (success)
        success = read_frame_data(frame);

    // read tail
    if (success)
        success = read_frame_tail(frame);

    // process data
    if (success) {
        // Serial.printf("Received frame with ID: %04X\n", frame.commandID);

        if (frame.commandID == 0x0301)
            packets_received++;

        FrameType type = static_cast<FrameType>(frame.commandID);
        
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
        case FrameType::PROJECTILE_SUPPLIER_STATUS:
            ref_data.projectile_supplier_status.set_data(frame.data);
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
        case FrameType::AIR_SUPPORT_STATUS:
            ref_data.air_support_status.set_data(frame.data);
            break;
        case FrameType::DAMAGE_STATUS:
            ref_data.damage_status.set_data(frame.data);
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
        case FrameType::KBM_INTERACTION:
            ref_data.kbm_interaction.set_data(frame.data);
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
        default:
            Serial.println("Unknown Frame Type");
            break;
        }
    }
}

void RefSystem::write(uint8_t* packet, uint8_t length) {
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

    // length of actual sendable data (without the IDs)
    uint8_t data_length = packet[1] - 6;

    // update header
    packet[0] = 0xA5;                       // set SOF
    packet[3] = get_seq();                  // set SEQ
    packet[4] = generateCRC8(packet, 4);    // set CRC

    // update sender ID
    packet[9] = ref_data.robot_performance.robot_ID;
    packet[10] = ref_data.robot_performance.robot_ID >> 8;

    // update tail
    uint16_t footerCRC = generateCRC16(packet, 13 + data_length);
    packet[13 + data_length] = (footerCRC & 0x00FF);    // set CRC
    packet[14 + data_length] = (footerCRC >> 8);        // set CRC

    // Serial.println("Attempting to send msg");
    if (Serial2.write(packet, length) == length) {
        packets_sent++;
        bytes_sent += length;
    } else
        Serial.println("Failed to write");
}

bool RefSystem::read_frame_header(Frame& frame) {
    // early return if Serial2 is empty or not full enough
    if (Serial2.available() < FrameHeader::packet_size)
        return false;

    // verify that what we are about to read is a header (first byte must be 0xA5)
    while (Serial2.peek() != 0xA5 && Serial2.peek() != -1) {
        Serial2.read();
    }

    // early return if Serial2 is empty or not full enough
    if (Serial2.available() < FrameHeader::packet_size)
        return false;

    // read and verify header
    int bytes_read = Serial2.readBytes(raw_buffer, FrameHeader::packet_size);
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
    if (frame.header.CRC != generateCRC8(raw_buffer, 4)) {
        Serial.println("Header failed CRC");
        packets_failed++;
        return false;
    }

    // increment buffer index
    buffer_index = bytes_read;

    return true;
}

bool RefSystem::read_frame_command_ID(Frame& frame) {
    // early return if Serial2 is empty or not full enough
    if (Serial2.available() < 2)
        return false;

    // read and verify command ID
    int bytes_read = Serial2.readBytes(raw_buffer + buffer_index, 2);
    if (bytes_read != 2) {
        Serial.println("Couldnt read enough bytes for ID");
        packets_failed++;
        return false;
    }

    // assign read ID
    frame.commandID = (raw_buffer[buffer_index + 1] << 8) | raw_buffer[buffer_index + 0];

    // sanity check, verify the ID is valid
    if (frame.commandID > REF_MAX_COMMAND_ID) {
        Serial.println("Invalid Command ID");
        packets_failed++;
        return false;
    }

    // increment buffer index
    buffer_index += bytes_read;

    return true;
}

bool RefSystem::read_frame_data(Frame& frame) {
    // early return if Serial2 is empty or not full enough
    if (Serial2.available() < frame.header.data_length)
        return false;

    // read and verify data
    int bytes_read = Serial2.readBytes(raw_buffer + buffer_index, frame.header.data_length);
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

bool RefSystem::read_frame_tail(Frame& frame) {
    // early return if Serial2 is empty or not full enough
    if (Serial2.available() < 2)
        return false;

    // read and verify tail
    int bytes_read = Serial2.readBytes(raw_buffer + buffer_index, 2);
    if (bytes_read != 2) {
        Serial.println("Couldnt read enough bytes for CRC");
        packets_failed++;
        return false;
    }

    // store CRC
    frame.CRC = (raw_buffer[buffer_index + 1] << 8) | raw_buffer[buffer_index + 0];

    if (frame.CRC != generateCRC16(raw_buffer, buffer_index)) {
        Serial.println("Tail failed CRC");
        packets_failed++;
        return false;
    }

    // increment buffer index
    buffer_index = bytes_read;

    return true;
}