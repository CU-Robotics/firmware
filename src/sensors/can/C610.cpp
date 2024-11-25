#include "C610.hpp"

template<typename CAN_BUS>
int C610<CAN_BUS>::read(CAN_message_t& msg) {
    // early return if the message ID does not match
    if (msg.id != m_base_id + m_id) return 0;

    // set m_input from msg
    memcpy(&m_input, &msg, sizeof(CAN_message_t));

    // fill out the motor state buffer
    m_state.torque = (m_input.buf[4] << 8) | m_input.buf[5];

    int16_t rpm = (m_input.buf[2] << 8) | m_input.buf[3];
    float rad_per_sec = rpm * ((2 * PI) / 60);
    m_state.speed = rad_per_sec;
    m_state.position = (m_input.buf[0] << 8) | m_input.buf[1];
    m_state.temperature = 0;    // we dont get temperature from C610
    
    return 1;
}

template<typename CAN_BUS>
int C610<CAN_BUS>::write(CAN_message_t& msg) {
    // TODO: does the caller need my local ID?

    // copy the internal m_output into msg
    memcpy(&msg, &m_output, sizeof(CAN_message_t));

    // TODO: can this fail?
    return 1;
}

template<typename CAN_BUS>
void C610<CAN_BUS>::write_motor_torque(float torque) {
    // clamp torque to -1 to 1 just in case. We dont want to overflow the int
    if (torque < -1.0f) torque = -1.0f;
    if (torque > 1.0f) torque = 1.0f;

    // map the normalized torque value to the actual torque value
    float mapped_torque = torque * m_max_torque;

    // convert given torque from float to 16-bit signed int
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

template<typename CAN_BUS>
void C610<CAN_BUS>::print_state() {
    Serial.printf("C610 Motor %d\n", m_id);
    Serial.printf("Torque: %d\n", m_state.torque);
    Serial.printf("Speed: %frad/s\n", m_state.speed);
    Serial.printf("Position: %d\n", m_state.position);
    Serial.printf("Temperature: %dC\n", m_state.temperature);
}

// explicitly declare the three possible types C610 can take

template class C610<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>>;
template class C610<FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>>;
template class C610<FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>>;