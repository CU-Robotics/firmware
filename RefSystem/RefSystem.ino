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

    if (Serial2.peek() == 0xA5)
    {
        ref.read(0x020C);
    }
    else
    {
        Serial2.flush();
        Serial2.clear();
    }

    // Serial.println("Bruh");
}