#include "C610.hpp"

void C610::init() {
    zero_motor();
}

int C610::read(CAN_message_t& msg) {
    // check the msg id and bus to see if this msg is for this motor
    if (!check_msg_id(msg)) return 0;

    // set m_input from msg
    memcpy(&m_input, &msg, sizeof(CAN_message_t));

    // fill out the motor state buffer
    // input data format is specified in the C610 datasheet (Speed Controller Sending Message Format)
    int16_t torque = (m_input.buf[4] << 8) | m_input.buf[5];
    m_state.torque = (float)torque / m_max_torque;

    int16_t rpm = (m_input.buf[2] << 8) | m_input.buf[3];
    float rad_per_sec = rpm * ((2 * PI) / 60);
    m_state.speed = rad_per_sec;
    m_state.position = (m_input.buf[0] << 8) | m_input.buf[1];
    m_state.temperature = 0;    // we dont get temperature from C610
    
    return 1;
}

int C610::write(CAN_message_t& msg) const {
    // set ID
    msg.id = m_output.id;

    // set only the buffer bytes that correspond to this motor
    // get the per-struct motor ID
    uint8_t motor_id = (m_id - 1) % 4;

    // fill in the output array
    // message format is specified in the C610 datasheet (Speed Controller Receiving Message Format)
    msg.buf[motor_id * 2] = m_output.buf[motor_id * 2];         // upper byte
    msg.buf[motor_id * 2 + 1] = m_output.buf[motor_id * 2 + 1]; // lower byte

    // return where in the buffer array the motor data was written
    // this is * 2 because we need the instance in this message where the first byte was edited. See the datasheet
    return motor_id * 2;
}

void C610::zero_motor() {
    // write 0 torque to the output msg
    write_motor_torque(0.0f);
}

void C610::write_motor_torque(float torque) {
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