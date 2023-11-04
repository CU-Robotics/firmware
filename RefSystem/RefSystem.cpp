#include "RefSystem.hpp"

uint8_t generateCRC8(uint8_t* data, uint32_t len)
{
    uint8_t CRC8 = 0xFF;
    while (len-- > 0)
    {
        uint8_t curr = CRC8 ^ (*data++);
        CRC8 = CRC8Lookup[curr];
    }
    return CRC8;
}

uint16_t generateCRC16(uint8_t* data, uint32_t len)
{
    uint16_t CRC16 = 0xFFFF;
    while (len-- > 0)
    {
        uint8_t curr = *data++;
        CRC16 = (CRC16 >> 8) ^ CRC16Lookup[(CRC16 ^ static_cast<uint16_t>(curr)) & 0x00FF];
    }
    return CRC16;
}


RefSystem::RefSystem()
{}

void RefSystem::init()
{
    Serial2.clear();
    Serial2.begin(112500);
    Serial2.flush();
    Serial2.clear();
}

void RefSystem::read(uint16_t filterID)
{
    Frame frame{};

    bool success = true;

    if (success)
        read_frame_header(frame);

    if (success)
        success = read_frame_command_ID(frame);

    if (success)
        success = read_frame_data(frame);

    if (success)
        success = read_frame_CRC(frame);

    if (success)
    {
        if (frame.commandID == 0x0301)
        {
            packets_received++;

        }

        switch (frame.commandID)
        {
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
        default:
            // Serial.println("What?");
            // frame.print();
            break;
        }
    }
}

void RefSystem::write(InterRobotComm message)
{
    if (bytes_sent >= 3720)
    {
        Serial.println("Too many bytes");
        return;
    }

    byte msg[128] = { 0 };
    int msg_len = 0;

    uint8_t data[message.size] = { 0 };
    for (int i = 0; i < message.size; i++)
        data[i] = message.data[i];

    msg[0] = 0xA5;
    msg[1] = 6 + message.size;
    msg[2] = 0x00;
    msg[3] = get_seq();
    msg[4] = generateCRC8(msg, 4);

    // cmd 0x0301
    msg[5] = 0x01;
    msg[6] = 0x03;

    // content ID
    msg[7] = message.content_id;
    msg[8] = message.content_id >> 8;

    // sender ID
    msg[9] = ref_data.robot_performance.robot_ID;
    msg[10] = ref_data.robot_performance.robot_ID >> 8;

    // receiver ID
    msg[11] = message.receiver_id;
    msg[12] = message.receiver_id >> 8;

    // data section
    for (int i = 0; i < message.size; i++)
        msg[13 + i] = data[i];

    uint16_t footerCRC = generateCRC16(msg, 13 + message.size);
    msg[13 + message.size] = (footerCRC & 0x00FF);
    msg[14 + message.size] = (footerCRC >> 8);

    msg_len = 15 + message.size;

    sent_sequence_queue[index++] = msg[3];

    // Serial.println("Attempting to send msg");
    if (Serial2.write(msg, msg_len) == msg_len)
    {
        packets_sent++;
        bytes_sent += msg_len;
    }
    else
        Serial.println("Failed to write");
}

bool RefSystem::read_frame_header(Frame& frame)
{
    int bytesRead = Serial2.readBytes(raw_buffer, FrameHeader::packet_size);
    if (bytesRead != FrameHeader::packet_size)
        return false;

    frame.header.SOF = raw_buffer[0];
    frame.header.data_length = (raw_buffer[2] << 8) | raw_buffer[1];
    frame.header.sequence = raw_buffer[3];
    frame.header.CRC = raw_buffer[4];

    if (frame.header.data_length > REF_MAX_PACKET_SIZE)
        return false;

    return true;
}

bool RefSystem::read_frame_command_ID(Frame& frame)
{
    int bytesRead = Serial2.readBytes(raw_buffer, 2);
    if (bytesRead != 2)
        return false;

    frame.commandID = (raw_buffer[1] << 8) | raw_buffer[0];
    if (frame.commandID > REF_MAX_COMMAND_ID)
        return false;

    return true;
}

bool RefSystem::read_frame_data(Frame& frame)
{
    memset(frame.data.data, 0, REF_MAX_PACKET_SIZE);

    int bytesRead = Serial2.readBytes(&frame.data.data[0], frame.header.data_length);
    if (bytesRead != frame.header.data_length)
        return false;

    return true;
}

bool RefSystem::read_frame_CRC(Frame& frame)
{
    int bytesRead = Serial2.readBytes(raw_buffer, 2);
    if (bytesRead != 2)
        return false;

    frame.CRC = (raw_buffer[1] << 8) | raw_buffer[0];

    return true;
}