#include "SDC302.hpp"
#include "FlexCAN_T4.h"
#include <cstdint>

void SDC302::init() {
    write_motor_on();
}

int SDC302::read(CAN_message_t& msg) {
    // check if the message is for this motor
    if (!check_msg_id(msg)) return 0;

    int16_t position = (msg.buf[1] << 8) | msg.buf[2];
    m_state.position = (position * 14.0 / 65535.0) - 7;

    int16_t speed = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
    m_state.speed = (speed * 300 / 4095 - 150) * 2 * M_PI / 60.0;

    int16_t torque = ((msg.buf[4] & 0x0F) << 8) | msg.buf[5];
    m_state.torque = torque * (400 * 0.68 * 6) / 4095 - 200 * 0.68 * 6;
    
    return 1;
}

int SDC302::write(CAN_message_t& msg) const {
    memcpy(&msg, &m_output, sizeof(CAN_message_t));

    return 0;
}

void SDC302::zero_motor() {
    write_motor_torque(0.0f);
}

void SDC302::write_motor_torque(float torque) {
    // clamp the torque value
    if (torque < -1.0f) torque = -1.0f;
    if (torque > 1.0f) torque = 1.0f;

    // map the -1f to 1f torque value to the actual torque value for this specific motor
    float mapped_torque = torque * m_max_torque;
    int16_t int_torque = (int16_t)mapped_torque;

    // create the CAN message
    CAN_message_t msg;
    create_cmd_control(msg, 0, 0, 0, 0, (((double)int_torque + 200 * 0.68 * 6) * 4095) / (400 * 0.68 * 6));
    Serial.printf("Torque: %d\n", int_torque);

    // fill in the output array
    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC302::print_state() const {
    Serial.printf("Bus: %x\tID: %x\tTemp: %.2dc\tTorque: % 4.3f%%\tSpeed: % 6.2frad/s\tPos: %5.5d?\n", m_bus_id, m_id, m_state.temperature, m_state.torque, m_state.speed, m_state.position);
}

void SDC302::write_motor_on() {
    CAN_message_t msg;
    create_cmd_motor_on(msg);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC302::write_motor_zero() {
    CAN_message_t msg;
    create_cmd_motor_zero(msg);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC302::write_motor_off() {
    CAN_message_t msg;
    create_cmd_motor_off(msg);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC302::create_cmd_motor_on(CAN_message_t& msg) {
    msg.id = m_id;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFC;  // FC
}

void SDC302::create_cmd_motor_zero(CAN_message_t& msg) {
    msg.id = m_id;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFE;  // FE
}

void SDC302::create_cmd_motor_off(CAN_message_t& msg) {
    msg.id = m_id;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFD;  // FD
}

void SDC302::create_cmd_control(CAN_message_t& msg, int16_t position, int16_t velocity, int16_t kp, int16_t kd, int16_t torque_ff) {
    msg.id = m_id;
    msg.buf[0] = (position >> 8) & 0xFF;
    msg.buf[1] = position & 0xFF;
    msg.buf[2] = ((velocity >> 4) & 0xFF);
    msg.buf[3] = ((velocity & 0x0F) << 4) | ((kp >> 8) & 0x0F);
    msg.buf[4] = kp & 0xFF;
    msg.buf[5] = (kd >> 4) & 0xFF;
    msg.buf[6] = ((kd & 0x0F) << 4) | ((torque_ff >> 8) & 0x0F);
    msg.buf[7] = torque_ff & 0xFF;
}