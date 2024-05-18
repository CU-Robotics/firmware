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

RefSystem::RefSystem() { }

void RefSystem::init() {
    // clear and start the MCM serial
    MCM_SERIAL.clear();
    MCM_SERIAL.begin(112500);
    MCM_SERIAL.flush();
    MCM_SERIAL.clear();

    // clear and start the VTM serial
    VTM_SERIAL.clear();
    VTM_SERIAL.begin(112500);
    VTM_SERIAL.flush();
    VTM_SERIAL.clear();
}

void RefSystem::read() {
    bool mcm_success = true;
    bool vtm_success = true;

    // reset buffer and index if we are reading a new packet
    if (!mcm_header_read) {
        memset(mcm_raw_buffer, 0, REF_MAX_PACKET_SIZE);
        mcm_buffer_index = 0;
    }
    if (!vtm_header_read) {
        memset(vtm_raw_buffer, 0, REF_MAX_PACKET_SIZE);
        vtm_buffer_index = 0;
    }

    // read header if we haven't already
    // mcm
    if (mcm_success && !mcm_header_read) {
        mcm_success = read_frame_header(&MCM_SERIAL, mcm_raw_buffer, mcm_buffer_index, mcm_curr_frame);
        mcm_header_read = mcm_success;
    }
    // vtm
    if (vtm_success && !vtm_header_read) {
        vtm_success = read_frame_header(&VTM_SERIAL, vtm_raw_buffer, vtm_buffer_index, vtm_curr_frame);
        vtm_header_read = vtm_success;
    }

    // read ID if we haven't already
    // mcm
    if (mcm_success && !mcm_command_ID_read) {
        mcm_success = read_frame_command_ID(&MCM_SERIAL, mcm_raw_buffer, mcm_buffer_index, mcm_curr_frame);
        mcm_command_ID_read = mcm_success;
    }
    // vtm
    if (vtm_success && !vtm_command_ID_read) {
        vtm_success = read_frame_command_ID(&VTM_SERIAL, vtm_raw_buffer, vtm_buffer_index, vtm_curr_frame);
        vtm_command_ID_read = vtm_success;
    }

    // read data if we haven't already
    // mcm
    if (mcm_success && !mcm_data_read) {
        mcm_success = read_frame_data(&MCM_SERIAL, mcm_raw_buffer, mcm_buffer_index, mcm_curr_frame);
        mcm_data_read = mcm_success;
    }
    // vtm
    if (vtm_success && !vtm_data_read) {
        vtm_success = read_frame_data(&VTM_SERIAL, vtm_raw_buffer, vtm_buffer_index, vtm_curr_frame);
        vtm_data_read = vtm_success;
    }

    // read tail if all other parts of the frame have been read
    // mcm
    if (mcm_success)
        mcm_success = read_frame_tail(&MCM_SERIAL, mcm_raw_buffer, mcm_buffer_index, mcm_curr_frame);
    // vtm
    if (vtm_success)
        vtm_success = read_frame_tail(&VTM_SERIAL, vtm_raw_buffer, vtm_buffer_index, vtm_curr_frame);

    // process data
    // mcm
    if (mcm_success) {
        set_ref_data(mcm_curr_frame, mcm_raw_buffer);
        // reset flags
        mcm_header_read = false;
        mcm_command_ID_read = false;
        mcm_data_read = false;
    }
    // vtm
    if (vtm_success) {
        set_ref_data(vtm_curr_frame, vtm_raw_buffer);
        // reset flags
        vtm_header_read = false;
        vtm_command_ID_read = false;
        vtm_data_read = false;
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
    if (Serial7.write(packet, length) == length) {
        packets_sent++;
        bytes_sent += length;
    } else
        Serial.println("Failed to write");
}

void RefSystem::get_data_for_comms(uint8_t output_array[180]) {
    // copys select packets into the output array
    memcpy(output_array + REF_COMMS_GAME_STATUS_OFFSET, ref_data.game_status.raw, ref_data.game_status.packet_size);
    memcpy(output_array + REF_COMMS_GAME_RESULT_OFFSET, ref_data.game_result.raw, ref_data.game_result.packet_size);
    memcpy(output_array + REF_COMMS_GAME_ROBOT_HP, ref_data.game_robot_hp.raw, ref_data.game_robot_hp.packet_size);
    memcpy(output_array + REF_COMMS_EVENT_DATE, ref_data.event_data.raw, ref_data.event_data.packet_size);
    memcpy(output_array + REF_COMMS_PROJECTILE_SUPPLIER_STATUS, ref_data.projectile_supplier_status.raw, ref_data.projectile_supplier_status.packet_size);
    memcpy(output_array + REF_COMMS_REFEREE_WARNING, ref_data.referee_warning.raw, ref_data.referee_warning.packet_size);
    memcpy(output_array + REF_COMMS_ROBOT_PERFORMANCE, ref_data.robot_performance.raw, ref_data.robot_performance.packet_size);
    memcpy(output_array + REF_COMMS_ROBOT_POWER_HEAT, ref_data.robot_power_heat.raw, ref_data.robot_power_heat.packet_size);
    memcpy(output_array + REF_COMMS_ROBOT_POSITION, ref_data.robot_position.raw, ref_data.robot_position.packet_size);
    memcpy(output_array + REF_COMMS_ROBOT_BUFF, ref_data.robot_buff.raw, ref_data.robot_buff.packet_size);
    memcpy(output_array + REF_COMMS_DAMAGE_STATUS, ref_data.damage_status.raw, ref_data.damage_status.packet_size);
    memcpy(output_array + REF_COMMS_LAUNCHING_STATUS, ref_data.launching_status.raw, ref_data.launching_status.packet_size);
    memcpy(output_array + REF_COMMS_PROJECTILE_ALLOWANCE, ref_data.projectile_allowance.raw, ref_data.projectile_allowance.packet_size);
    memcpy(output_array + REF_COMMS_RFID_STATUS, ref_data.rfid_status.raw, ref_data.rfid_status.packet_size);
    memcpy(output_array + REF_COMMS_KBM_INTERACTION, ref_data.kbm_interaction.raw, ref_data.kbm_interaction.packet_size);
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
    if (frame.header.CRC != generateCRC8(raw_buffer, 4)) {
        Serial.println("Header failed CRC");
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
        Serial.println("Invalid Command ID");
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

bool RefSystem::read_frame_tail(HardwareSerial* serial, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2], uint16_t& buffer_index, Frame& frame) {
    // early return if serial is empty or not full enough
    if (serial->available() < 2)
        return false;

    // read and verify tail
    int bytes_read = serial->readBytes(raw_buffer + buffer_index, 2);
    if (bytes_read != 2) {
        Serial.println("Couldnt read enough bytes for CRC");
        packets_failed++;
        return false;
    }

    // store CRC
    frame.CRC = (raw_buffer[buffer_index + 1] << 8) | raw_buffer[buffer_index + 0];

    if (frame.CRC != generateCRC16(raw_buffer, buffer_index)) {
        // Serial.println("Tail failed CRC");
        packets_failed++;
        return false;
    }

    // increment buffer index
    buffer_index = bytes_read;

    return true;
}

void RefSystem::set_ref_data(Frame& frame, uint8_t raw_buffer[REF_MAX_PACKET_SIZE * 2]) {
    // Serial.printf("Received frame with ID: %04X\n", *reinterpret_cast<uint16_t*>(raw_buffer + 5));

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
        // ref_data.kbm_interaction.print();
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
