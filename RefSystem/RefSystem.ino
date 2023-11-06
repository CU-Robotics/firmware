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

    if (accumulator > 40000)
    {
        accumulator = 0;
        ref.write();
    }

    ref.read();

    Serial.printf("S: %u\tR: %u\tM: %u\t%.4f\tA: %u\n", ref.packets_sent, ref.packets_received, ref.packets_sent - ref.packets_received, (float)ref.packets_received / (float)ref.packets_sent, Serial2.available());
}