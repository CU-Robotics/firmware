#include "RefSystem.hpp"

RefSystem ref;

uint32_t accumulator = 0;
uint32_t second_accumulator = 0;
uint32_t dt = 0;
uint32_t curr_time = 0;

uint8_t data[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1 };

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
    second_accumulator += dt;

    if (second_accumulator > 1000000u)
    {
        ref.bytes_sent = 0;
        second_accumulator = 0;
    }

    if (accumulator > 35000)
    {
        accumulator = 0;
        InterRobotComm msg{};
        msg.content_id = 0x0201;
        msg.receiver_id = 0x0007;
        msg.set_data(data, 75);
        ref.write(msg);
    }

    ref.read();

    Serial.printf("S: %u\tR: %u\t\t%.2f\n", ref.packets_sent, ref.packets_received, (float)ref.packets_received / (float)ref.packets_sent);
}