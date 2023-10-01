#include "DR16.hpp"



DR16::DR16() {}

void DR16::Init()
{
    Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);
    Serial5.flush();
    Serial5.clear();

    for (int i = 0; i < DR16_PACKET_SIZE; i++)
    {
        m_inputRaw[i] = 0;
    }

    for (int i = 0; i < DR16_INPUT_VALUE_COUNT; i++)
    {
        m_input[i] = 0;
    }
}

void DR16::Read()
{
    uint16_t c0{ 0 }, c1{ 0 }, c2{ 0 }, c3{ 0 }, wheel{ 0 };
    uint8_t s1{ 0 }, s2{ 0 };

    // read
    if (Serial5.available() > 18)
    {
      Serial5.clear();
      return;
    }
    if (Serial5.available() < 18)
    {
      return;
    }
    
    Serial5.flush();
    Serial5.readBytes(m_inputRaw, 18);
    Serial5.clear();

    // set channel values
    c0 = ((m_inputRaw[1] & 0x7) << 8) | m_inputRaw[0];
    c1 = ((m_inputRaw[2] & 0x3f) << 5) | ((m_inputRaw[1] & 0xf8) >> 3);
    c2 = ((m_inputRaw[4] & 0x01) << 10) | ((m_inputRaw[3] & 0xff) << 2) | ((m_inputRaw[2] & 0xc0) >> 6);
    c3 = ((m_inputRaw[5] & 0x0f) << 7) | ((m_inputRaw[4] & 0xfe) >> 1);
    s1 = (m_inputRaw[5] & 0x30) >> 4;
    s2 = (m_inputRaw[5] & 0xc0) >> 6;
    wheel = ((m_inputRaw[17] & 0x7) << 8) | m_inputRaw[16];

    if (IsDataValid())
    {
        // joy sticks
        m_input[0] = bounded_map(c0, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
        m_input[1] = bounded_map(c1, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
        m_input[2] = bounded_map(c2, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;
        m_input[3] = bounded_map(c3, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;

        // wheel
        m_input[4] = bounded_map(wheel, DR16_CONTROLLER_INPUT_LOW, DR16_CONTROLLER_INPUT_HIGH, -1000, 1000) / 1000.f;

        // switches
        m_input[5] = (float)s1;
        m_input[6] = (float)s2;
    }

    // Serial.printf("0: %d\t1: %d\t2: %d\t3: %d\ts1: %d\ts2: %d\twheel: %d\n", c0, c1, c2, c3, s1, s2, wheel);

    // for (int i = 0; i < DR16_PACKET_SIZE; i++)
    //     Serial.printf("%x\t", m_inputRaw[i]);
    // Serial.println();
    Print();
}

float* DR16::GetInput()
{
    return nullptr;
}

void DR16::Print()
{
    Serial.printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", m_input[0], m_input[1], m_input[2], m_input[3], m_input[4], m_input[5], m_input[6]);
}

float DR16::bounded_map(int value, int in_low, int in_high, int out_low, int out_high)
{
    /*
        This is derived from sthe arduino map() function.
    */
    value = max(min(value, in_high), in_low);
    return (value - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}
