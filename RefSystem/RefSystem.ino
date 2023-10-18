#include "RefSystem.hpp"

RefSystem ref;

void setup()
{
    Serial.begin(9800);
    Serial.println("Start");
    ref.Init();
    Serial.println("Init");
}

void loop()
{
    // delay(1);

    if (Serial2.peek() == 0xA5)
    {
        bool success = true;

        Frame frame{};
        success = ref.ReadFrameStart(frame);

        if (success)
            success = ref.ReadFrameCommandID(frame);

        if (success)
            success = ref.ReadFrameData(frame);

        if (success)
            success = ref.ReadFrameEnd(frame);

        if (success)
        {
            frame.printType();
        }
    }
    else
    {
        Serial2.flush();
        Serial2.clear();
    }

    // Serial.println("Bruh");
}