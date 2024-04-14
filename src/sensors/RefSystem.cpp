#include "RefSystem.hpp"

uint8_t generateCRC8(uint8_t* data, uint32_t len)
{
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
    Frame frame{};

    bool success = true;

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
        if (frame.commandID == 0x0301)
            packets_received++;

        switch (frame.commandID) {
        case GAME_STATUS:
            ref_data.game_status.initialize_from_data(frame.data);
            break;
        case GAME_RESULT:
            ref_data.game_result.initialize_from_data(frame.data);
            break;
        case ROBOT_HEALTH:
            ref_data.robot_health.initialize_from_data(frame.data);
            break;
        case SITE_EVENT:
            ref_data.site_event.initialize_from_data(frame.data);
            break;
        case PROJECTILE_SUPPLIER:
            ref_data.proj_supplier.initialize_from_data(frame.data);
            break;
        case REFEREE_WARNING:
            ref_data.ref_warning.initialize_from_data(frame.data);
            break;
        case DART_LAUNCH:
            ref_data.dart_launch.initialize_from_data(frame.data);
            break;
        case ROBOT_PERFORMANCE:
            ref_data.robot_performance.initialize_from_data(frame.data);
            break;
        case POWER_HEAT:
            ref_data.power_heat.initialize_from_data(frame.data);
            break;
        case ROBOT_POSITION:
            ref_data.position.initialize_from_data(frame.data);
            break;
        case ROBOT_BUFF:
            ref_data.robot_buff.initialize_from_data(frame.data);
            break;
        case AIR_SUPPORT_TIME:
            ref_data.air_support_time.initialize_from_data(frame.data);
            break;
        case DAMAGE_STATUS:
            ref_data.damage_status.initialize_from_data(frame.data);
            break;
        case LAUNCHING_EVENT:
            ref_data.launching_event.initialize_from_data(frame.data);
            break;
        case PROJECTILE_ALLOWANCE:
            ref_data.proj_allowance.initialize_from_data(frame.data);
            break;
        case RFID:
            ref_data.rfid.initialize_from_data(frame.data);
            break;
        case DART_COMMAND:
            ref_data.dart_command.initialize_from_data(frame.data);
            break;
        case GROUND_ROBOT_POSITION:
            ref_data.ground_positions.initialize_from_data(frame.data);
            break;
        case RADAR_PROGRESS:
            ref_data.radar_progress.initialize_from_data(frame.data);
            break;
        case INTER_ROBOT_COMM:
            ref_data.inter_robot_comms[inter_robot_comm_index].initialize_from_data(frame);
            inter_robot_comm_index++;
            if (inter_robot_comm_index >= REF_MAX_COMM_BUFFER_SIZE)
                inter_robot_comm_index = 0;
            break;
        default:
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
    }
    else
        Serial.println("Failed to write");
}

bool RefSystem::read_frame_header(Frame& frame) {
    // early return if Serial2 is empty or not full enough
    // Serial.println(Serial2.available());
    
    if (Serial2.available() < FrameHeader::packet_size) {
        return false;
    }
    
    if (Serial2.peek() != 0xA5) {
        Serial2.read();
        return false;
    }
    
    // read and verify header
    int bytesRead = Serial2.readBytes(raw_buffer, FrameHeader::packet_size);
    if (bytesRead != FrameHeader::packet_size) {
        Serial.println("Couldnt read enough bytes for Header");
        packets_failed++;
        return false;
    }

    // set read data
    frame.header.SOF = raw_buffer[0];
    if (frame.header.SOF != 0xA5) {
        Serial.println("Not a valid frame");
        return false;
    }
    
    frame.header.data_length = (raw_buffer[2] << 8) | raw_buffer[1];
    frame.header.sequence = raw_buffer[3];
    frame.header.CRC = raw_buffer[4];

    // verify the CRC is correct
    if (frame.header.CRC != generateCRC8(raw_buffer, 4)) {
        Serial.println("Header failed CRC");
        packets_failed++;
        return false;
    }

    return true;
}

bool RefSystem::read_frame_command_ID(Frame& frame) {
    // early return if Serial2 is empty or not full enough
    if (Serial2.available() < 2)
        return false;

    // read and verify command ID
    int bytesRead = Serial2.readBytes(raw_buffer, 2);
    if (bytesRead != 2) {
        Serial.println("Couldnt read enough bytes for ID");
        packets_failed++;
        return false;
    }

    // assign read ID
    frame.commandID = (raw_buffer[1] << 8) | raw_buffer[0];

    // sanity check, verify the ID is valid
    if (frame.commandID > REF_MAX_COMMAND_ID) {
        Serial.println("Invalid Command ID");
        packets_failed++;
        return false;
    }

    return true;
}

bool RefSystem::read_frame_data(Frame& frame) {
    // early return if Serial2 is empty or not full enough
    if (Serial2.available() < frame.header.data_length)
        return false;
    
    // read and verify data
    int bytesRead = Serial2.readBytes(&frame.data.data[0], frame.header.data_length);
    if (bytesRead != frame.header.data_length) {
        Serial.println("Couldnt read enough bytes for Data");
        packets_failed++;
        return false;
    }

    return true;
}

bool RefSystem::read_frame_tail(Frame& frame) {
    // early return if Serial2 is empty or not full enough
    if (Serial2.available() < 2)
        return false;
    
    // read and verify tail
    int bytesRead = Serial2.readBytes(raw_buffer, 2);
    if (bytesRead != 2) {
        Serial.println("Couldnt read enough bytes for CRC");
        packets_failed++;
        return false;
    }

    // store CRC
    frame.CRC = (raw_buffer[1] << 8) | raw_buffer[0];

    return true;
}