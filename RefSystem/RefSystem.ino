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
    }
    else
    {
        Serial2.flush();
        Serial2.clear();
    }

    // Serial.println("Bruh");
}