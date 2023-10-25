#include "RefSystem.hpp"

RefSystem ref;

void setup()
{
    Serial.begin(9800);
    Serial.println("Start");
    ref.init();
    Serial.println("Init");
}

void loop()
{
    // delay(1);

        Frame frame{};
        frame.header.SOF = 0xA5;
        frame.header.data_length = 32;
        frame.header.sequence = 5;
        frame.header.CRC = 1;

        frame.commandID = 0x0301;

        FrameData data{};
        data.data[0] = 0x01;
        data.data[1] = 0x02;
        data.data[2] = 7;
        data.data[3] = 0;
        data.data[4] = 7;
        data.data[5] = 0;

        for (int i = 0; i < 32 - 6; i++)
        {
            data.data[6 + i] = i;
        }

        frame.data = data;

        frame.CRC = 2;

        ref.write(frame);
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