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
        ref.write(frame);
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