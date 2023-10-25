#include "RefSystem.hpp"

RefSystem ref;

uint32_t accumulator = 0;
uint32_t dt = 0;
uint32_t curr_time = 0;

void setup()
{
    Serial.begin(9800);
    Serial.println("Start");
    ref.init();
    Serial.println("Init");
    curr_time = micros();
}

void loop()
{
    // delay(1);
    dt = micros() - curr_time;
    curr_time = micros();
    accumulator += dt;

    if (accumulator >= 30000)
    {
        accumulator = 0;

        Frame frame{};
        frame.header.SOF = 0xA5;
        frame.header.data_length = 32;
        frame.header.sequence = 5;
        frame.header.CRC = 0;

        frame.commandID = 0x0301;

        FrameData data{};
        data.data[0] = 0x01;    // Sub-content ID
        data.data[1] = 0x02;
        data.data[2] = 1;       // Sender ID
        data.data[3] = 0;
        data.data[4] = 7;       // Receiver ID
        data.data[5] = 0;

        // raw data content
        for (int i = 0; i < 32 - 6; i++)
        {
            data.data[6 + i] = i;
        }

        frame.data = data;

        frame.CRC = 0;  // figure out crc in write command

        // ref.write(frame);
    }


    if (Serial2.peek() == 0xA5)
    {
        ref.read(0x0301);
    }
    else
    {
        Serial2.flush();
        Serial2.clear();
    }

    // Serial.println("Bruh");
}