#include "RefSystem.hpp"

RefSystem::RefSystem()
{}

void RefSystem::Init()
{
    Serial2.begin(115200);
    Serial2.clear();
}

bool RefSystem::ReadFrameStart(Frame& frame)
{
    int numBytesRead = Serial2.readBytes(m_rawData, FrameHeader::size);
    if (numBytesRead != FrameHeader::size)
    {
        Serial.println("Failed to read enough bytes for FrameHeader");
        frame.header.SOF = 0x0;
        return false;
    }

    frame.header.SOF = m_rawData[0];
    frame.header.data_length = (m_rawData[2] << 8) | m_rawData[1];
    frame.header.seq = m_rawData[3];
    frame.header.CRC8 = m_rawData[4];

    if (frame.header.data_length > REF_MAX_FRAME_DATA_SIZE)
    {
        Serial.println("Failed to read valid FrameHeader");
        // Serial2.clear();
        frame.header.SOF = 0x0;
        return false;
    }

    // Serial.printf("FrameHeader: %.2x %d %.2x %.2x\n", frame.header.SOF, frame.header.data_length, frame.header.seq, frame.header.CRC8);

    return true;
}

bool RefSystem::ReadFrameCommandID(Frame& frame)
{
    int numBytes = Serial2.readBytes(m_rawData, 2);
    if (numBytes != 2)
    {
        Serial.println("Failed to read enough bytes for Command ID");
        frame.cmdID = 0;
        return false;
    }

    frame.cmdID = (m_rawData[1] << 8) | m_rawData[0];

    if (frame.cmdID > REF_MAX_COMMAND_ID || frame.cmdID == 0)
    {
        Serial.println("Invalid Command ID Found");
        frame.cmdID = 0;
        return false;
    }

    // Serial.printf("Command ID: %.4x\n", frame.cmdID);

    return true;
}

bool RefSystem::ReadFrameData(Frame& frame)
{
    memset(&frame.data.data[0], 0, REF_MAX_FRAME_DATA_SIZE);

    int numBytes = Serial2.readBytes(&frame.data.data[0], frame.header.data_length);
    if (numBytes != frame.header.data_length)
    {
        Serial.println("Failed to read data from current frame");
        return false;
    }

    // Serial.println("Frame Data: ");
    // for (int i = 0; i < frame.header.data_length; i++)
    // {
    //     Serial.printf("%.2x ", frame.data.data[i]);
    //     if (i % 16 == 15)
    //         Serial.println();
    // }
    // Serial.println();

    return true;
}

bool RefSystem::ReadFrameEnd(Frame& frame)
{
    int numBytes = Serial2.readBytes(m_rawData, 2);
    if (numBytes != 2)
    {
        Serial.println("Failed to read enough bytes for FrameEnd");
        frame.CRC16 = 0;
        return false;
    }

    frame.CRC16 = (m_rawData[1] << 8) | m_rawData[0];

    // Serial.printf("Frame End CRC16: %.4x\n", frame.CRC16);

    return true;
}
