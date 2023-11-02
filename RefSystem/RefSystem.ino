#include "RefSystem.hpp"

RefSystem ref;

uint32_t accumulator = 0;
uint32_t dt = 0;
uint32_t curr_time = 0;

uint8_t data[5] = { 1, 2, 3, 4, 5 };

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
    dt = micros() - curr_time;
    curr_time = micros();
    accumulator += dt;

    if (accumulator >= 30000)
    {
        accumulator = 0;
        InterRobotComm msg{};
        msg.content_id = 0x0201;
        msg.receiver_id = 0x0001;
        msg.set_data(data, 5);
        ref.write(msg);
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