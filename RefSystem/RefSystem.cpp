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
        if (frame.commandID == 0x0201)
        {
            RobotPerformance status;
            status.initialize_from_data(frame.data);
            ID = status.robot_ID;
        }
        else if (filterID == frame.commandID)
        {
            frame.print();
        }
    }
}

void RefSystem::write(Frame& frame)
{
    // generate raw packet
    uint16_t size = 0;
    uint8_t packet[REF_MAX_PACKET_SIZE] = { 0 };

    // write frame header
    packet[0] = frame.header.SOF;
    packet[1] = frame.header.data_length;
    packet[2] = frame.header.data_length >> 8;
    packet[3] = frame.header.sequence;
    packet[4] = generateCRC8(packet, 4);
    size += 5;

    // write frame command ID
    packet[5] = frame.commandID;
    packet[6] = frame.commandID >> 8;
    size += 2;

    // write frame data
    for (int i = 0; i < frame.header.data_length; i++)
    {
        packet[size + i] = frame.data[i];
    }
    size += frame.header.data_length;

    // write frame CRC
    uint16_t CRC16 = generateCRC16(packet, size);
    packet[size + 1] = CRC16;
    packet[size + 2] = CRC16 >> 8;
    size += 2;

    Serial.println("Attempting to write: ");
    for (int i = 0; i < size; i++)
        Serial.printf("%x ", packet[i]);
    Serial.println();

    // issue write command
    Serial2.write(packet, size);
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
