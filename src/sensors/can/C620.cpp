#include "C620.hpp"

template<typename CAN_BUS>
int C620<CAN_BUS>::read(CAN_message_t& msg) {
    // set m_input from msg
    memcpy(&m_input, &msg, sizeof(CAN_message_t));

    return 1;
}

template<typename CAN_BUS>
int C620<CAN_BUS>::write(CAN_message_t& msg) {
    // TODO: does the caller need my local ID?

    // copy the internal m_output into msg
    memcpy(&msg, &m_output, sizeof(CAN_message_t));

    return 1;
}

template<typename CAN_BUS>
void C620<CAN_BUS>::write_motor_torque(float torque) {
    // clamp torque to -1 to 1 just in case. We dont want to overflow the int
    if (torque < -1.0f) torque = -1.0f;
    if (torque > 1.0f) torque = 1.0f;

    // convert given torque from float to 16-bit signed int
    float mapped_torque = torque * m_max_torque;

    int16_t int_torque = (int16_t)mapped_torque;

    // map the ID
    uint8_t message_id = (m_id - 1) / 4;

    // set the output ID
    if (message_id == 0)
        m_output.id = 0x200;    // first 4 motors
    else
        m_output.id = 0x1FF;    // last 4 motors

    // get the per-struct motor ID
    uint8_t motor_id = (m_id - 1) % 4;

    // fill in the output array
    m_output.buf[motor_id * 2] = (int_torque >> 8) & 0xFF;  // upper byte
    m_output.buf[motor_id * 2 + 1] = int_torque & 0xFF;     // lower byte
}

// explicitly declare the three possible types C620 can take

template class C620<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>>;
template class C620<FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>>;
template class C620<FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>>;