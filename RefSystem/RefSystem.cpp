#include "RefSystem.hpp"

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
    uint8_t packet[REF_MAX_PACKET_SIZE] = { 0 };

    // write frame header
    packet[0] = frame.header.SOF;
    packet[1] = frame.header.data_length & 0x00ff;
    packet[2] = (frame.header.data_length & 0xff00) >> 8;
    packet[3] = frame.header.sequence;
    packet[4] = frame.header.CRC;

    // write frame command ID
    packet[5] = frame.commandID & 0x00ff;
    packet[6] = (frame.commandID & 0xff00) >> 8;

    // write frame data
    for (int i = 0; i < REF_MAX_PACKET_SIZE - 7; i++)
    {
        packet[7 + i] = frame.data[i];
    }

    // write frame CRC
    packet[REF_MAX_PACKET_SIZE - 2] = frame.CRC & 0x00ff;
    packet[REF_MAX_PACKET_SIZE - 1] = (frame.CRC & 0xff00) >> 8;

    // issue write command
    Serial2.write(packet, frame.header.data_length + FrameHeader::packet_size + 4);
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
