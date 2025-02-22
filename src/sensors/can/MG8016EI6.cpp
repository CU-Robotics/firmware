#include "MG8016EI6.hpp"

void MG8016EI6::init() {
    write_motor_on();
}

int MG8016EI6::read(CAN_message_t& msg) {
    // command IDs that return the base state (temp, torque, speed, angle)
    // A1, A2, A3, A4, A5, A6, A7, A8, 9C (state 2)
    // CMD_TORQUE_CONTROL, CMD_SPEED_CONTROL, CMD_MULTI_ANGLE_CONTROL, CMD_MULTI_ANGLE_CONTROL_SPEED_LIMITED,
    // CMD_ANGLE_CONTROL, CMD_ANGLE_CONTROL_SPEED_LIMITED, CMD_ANGLE_INCREMENT_CONTROL, 
    // CMD_ANGLE_INCREMENT_CONTROL_SPEED_LIMITED, CMD_READ_STATE_2

    // command IDs that return special state (current, error, etc)
    // 30, 33, 90, 19, 92 (multi angle), 94 (single angle), 9A/9B (state 1), 9D (state 3)
    // CMD_READ_PID, CMD_READ_ACCELERATION, CMD_READ_ENCODER, CMD_READ_MULTI_ANGLE,
    // CMD_READ_ANGLE, CMD_READ_STATE_1, CMD_READ_STATE_3

    // command IDs that dont return anything other than the command echo'd back
    // 80, 88, 81, 31, 32, 34, 91
    // CMD_MOTOR_OFF, CMD_MOTOR_ON, CMD_MOTOR_STOP, CMD_WRITE_PID, CMD_WRITE_PID_ROM, CMD_WRITE_ACCELERATION, CMD_WRITE_ENCODER_ZERO

    // check the msg id and bus to see if this msg is for this motor
    if (!check_msg_id(msg)) return 0;

    uint8_t cmd_byte = msg.buf[0];

    // the data format of each of these commands is specified in the MG8016E-i6 CAN protocol datasheet, it is referred to as "Driver Respond"

    // handle the command
    switch (cmd_byte) {
    // commands that return the base state
    case CMD_TORQUE_CONTROL:
    case CMD_SPEED_CONTROL:
    case CMD_MULTI_ANGLE_CONTROL:
    case CMD_MULTI_ANGLE_CONTROL_SPEED_LIMITED:
    case CMD_ANGLE_CONTROL:
    case CMD_ANGLE_CONTROL_SPEED_LIMITED:
    case CMD_ANGLE_INCREMENT_CONTROL:
    case CMD_ANGLE_INCREMENT_CONTROL_SPEED_LIMITED:
    case CMD_READ_STATE_2: {
        // read the state
        m_state.temperature = msg.buf[1];
        // torque is given in the signed range [-2048, 2048] which corresponds to [-33A, 33A]
        // normalize this to [-1, 1]
        int16_t raw_torque = (int16_t)((msg.buf[3] << 8) | msg.buf[2]);
        m_state.torque = (float)raw_torque / m_max_torque;
        
        // speed is given in 1dps/LSB
        // need to conver this to rad/s
        int16_t speed_dps = (int16_t)((msg.buf[5] << 8) | msg.buf[4]);
        float speed_rad = speed_dps * (PI / 180);
        m_state.speed = speed_rad;
        m_state.position = (uint16_t)((msg.buf[7] << 8) | msg.buf[6]);
        break;
    }
    // commands that return special state
    case CMD_READ_PID: {
        m_angle_p = msg.buf[2];
        m_angle_i = msg.buf[3];
        m_speed_p = msg.buf[4];
        m_speed_i = msg.buf[5];
        m_torque_p = msg.buf[6];
        m_torque_i = msg.buf[7];
        break;
    }
    case CMD_READ_ACCELERATION: {
        m_acceleration = (int32_t)((msg.buf[7] << 24) | (msg.buf[6] << 16) | (msg.buf[5] << 8) | msg.buf[4]);
        break;
    }
    case CMD_READ_ENCODER: {
        m_encoder = (uint16_t)((msg.buf[3] << 8) | msg.buf[2]);
        m_encoder_raw = (uint16_t)((msg.buf[5] << 8) | msg.buf[4]);
        m_encoder_offset = (uint16_t)((msg.buf[7] << 8) | msg.buf[6]);
        break;
    }
    case CMD_READ_MULTI_ANGLE: {
        // TODO: Does this work?
        
        // directly assign the first 7 bytes from the msg buffer (skipping the command byte)
        // direct assignment due to weirdness with bitshifts on values larger than the word size
        for (int i = 0; i < 7; i++) {
            *((uint8_t*)(&m_multi_motor_mode_angle) + i) = msg.buf[i + 1];
        }

        // sign extend the last byte if needed
        // we do this because we're given a 7 byte signed value, but directly filling a 8 byte value
        // would mess up it's value since the sign bit would not be preserved
        // this essentially performs an arithmetic right shift in a defined way
        if (msg.buf[7] & 0x80) {
            *((uint8_t*)(&m_multi_motor_mode_angle) + 7) = 0xFF;
        } else {
            *((uint8_t*)(&m_multi_motor_mode_angle) + 7) = 0x00;
        }
        
        break;
    }
    case CMD_READ_ANGLE: {
        m_single_motor_mode_angle = (uint32_t)((msg.buf[7] << 24) | (msg.buf[6] << 16) | (msg.buf[5] << 8) | msg.buf[4]);
        break;
    }
    case CMD_READ_STATE_1:
    case CMD_CLEAR_ERROR: {
        m_voltage = (uint16_t)((msg.buf[4] << 8) | msg.buf[3]);
        m_error_state.under_voltage = !!(msg.buf[7] & 0x01);
        m_error_state.over_temperature = !!(msg.buf[7] & 0x08);
        break;
    }
    case CMD_READ_STATE_3: {
        m_a_phase_current = (int16_t)((msg.buf[3] << 8) | msg.buf[2]);
        m_b_phase_current = (int16_t)((msg.buf[5] << 8) | msg.buf[4]);
        m_c_phase_current = (int16_t)((msg.buf[7] << 8) | msg.buf[6]);
        break;
    }
    // commands that dont return anything
    case CMD_MOTOR_OFF:
    case CMD_MOTOR_ON:
    case CMD_MOTOR_STOP:
    case CMD_WRITE_PID:
    case CMD_WRITE_PID_ROM:
    case CMD_WRITE_ACCELERATION:
    case CMD_WRITE_ENCODER_ZERO: {
        // do nothing
        break;
    }
    default:
        Serial.printf("Unknown command byte: 0x%02X\n", cmd_byte);
        break;
    }

    return 1;
}

int MG8016EI6::write(CAN_message_t& msg) const {
    memcpy(&msg, &m_output, sizeof(CAN_message_t));

    return 0;
}

void MG8016EI6::write_motor_torque(float torque) {
    // Clamp the torque value
    if (torque > 1.0f) {
        torque = 1.0f;
    } else if (torque < -1.0f) {
        torque = -1.0f;
    }

    // Convert the torque value to the motor's torque range
    int16_t torque_val = (int16_t)(torque * m_max_torque);

    // Create the command
    uint8_t buf[8];
    create_cmd_torque_control(buf, torque_val);

    // fill in the output array
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void MG8016EI6::zero_motor() {
    // write 0 torque to the output msg
    write_motor_torque(0.0f);
}

void MG8016EI6::write_motor_speed(float speed) {
    // convert given speed in rad/s to 0.01dps/LSB

    // convert from rad/s to dps
    speed *= (180 / PI);

    // convert from dps to 0.01dps
    speed *= 100;

    // cast to int32_t
    int32_t speed_val = (int32_t)(speed);

    // Create the command
    uint8_t buf[8];
    create_cmd_speed_control(buf, speed_val);

    // fill in the output array
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void MG8016EI6::write_motor_angle(float angle, float speed_limit) {
    // convert given angle in rad to 0.01deg/LSB

    // convert from rad to deg
    angle *= (180 / PI);

    // convert from deg to 0.01deg
    angle *= 100;

    // cast to int32_t
    int32_t angle_val = (int32_t)(angle);

    // make sure speed_limit is positive
    if (speed_limit < 0) {
        speed_limit = -speed_limit;
    }

    // convert speed_limit from rad/s to 1dps/LSB
    // convert from rad/s to dps
    speed_limit *= (180 / PI);

    // convert from dps to 1dps
    speed_limit *= 1;

    // Create the command
    uint8_t buf[8];
    if (speed_limit == 0) { // no speed limit
        create_cmd_multi_angle_control(buf, angle_val);
    } else {    // speed limit
        create_cmd_multi_angle_control_speed_limited(buf, angle_val, (uint16_t)speed_limit);
    }

    // fill in the output array
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void MG8016EI6::write_motor_off() {
    // create the message
    uint8_t buf[8];
    create_cmd_motor_off(buf);

    // fill in the output array
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void MG8016EI6::write_motor_on() {
    // create the message
    uint8_t buf[8];
    create_cmd_motor_on(buf);

    // fill in the output array
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void MG8016EI6::write_motor_stop() {
    // create the message
    uint8_t buf[8];
    create_cmd_motor_stop(buf);

    // fill in the output array
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

// Command creation functions

void MG8016EI6::create_cmd_motor_off(uint8_t buf[8]) {
    buf[0] = CMD_MOTOR_OFF;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_motor_on(uint8_t buf[8]) {
    buf[0] = CMD_MOTOR_ON;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_motor_stop(uint8_t buf[8]) {
    buf[0] = CMD_MOTOR_STOP;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_torque_control(uint8_t buf[8], int16_t torque) {
    buf[0] = CMD_TORQUE_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&torque) + 0);         // low byte
    buf[5] = *((uint8_t*)(&torque) + 1);         // high byte
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_speed_control(uint8_t buf[8], int32_t speed) {
    buf[0] = CMD_SPEED_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&speed) + 0);         // low byte
    buf[5] = *((uint8_t*)(&speed) + 1);
    buf[6] = *((uint8_t*)(&speed) + 2);
    buf[7] = *((uint8_t*)(&speed) + 3);         // high byte
}

void MG8016EI6::create_cmd_multi_angle_control(uint8_t buf[8], int32_t angle) {
    buf[0] = CMD_MULTI_ANGLE_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&angle) + 0);         // low byte
    buf[5] = *((uint8_t*)(&angle) + 1);
    buf[6] = *((uint8_t*)(&angle) + 2);
    buf[7] = *((uint8_t*)(&angle) + 3);         // high byte
}

void MG8016EI6::create_cmd_multi_angle_control_speed_limited(uint8_t buf[8], int32_t angle, uint16_t speed_limit) {
    buf[0] = CMD_MULTI_ANGLE_CONTROL_SPEED_LIMITED;
    buf[1] = 0;
    buf[2] = *((uint8_t*)(&speed_limit) + 0);   // low byte
    buf[3] = *((uint8_t*)(&speed_limit) + 1);   // high byte
    buf[4] = *((uint8_t*)(&angle) + 0);         // low byte
    buf[5] = *((uint8_t*)(&angle) + 1);
    buf[6] = *((uint8_t*)(&angle) + 2);
    buf[7] = *((uint8_t*)(&angle) + 3);         // high byte
}

void MG8016EI6::create_cmd_angle_control(uint8_t buf[8], uint32_t angle, uint8_t spin_direction) {
    buf[0] = CMD_ANGLE_CONTROL;
    buf[1] = spin_direction;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&angle) + 0);         // low byte
    buf[5] = *((uint8_t*)(&angle) + 1);
    buf[6] = *((uint8_t*)(&angle) + 2);
    buf[7] = *((uint8_t*)(&angle) + 3);         // high byte
}

void MG8016EI6::create_cmd_angle_control_speed_limited(uint8_t buf[8], uint32_t angle, uint8_t spin_direction, uint16_t speed_limit) {
    buf[0] = CMD_ANGLE_CONTROL_SPEED_LIMITED;
    buf[1] = spin_direction;
    buf[2] = *((uint8_t*)(&speed_limit) + 0);   // low byte
    buf[3] = *((uint8_t*)(&speed_limit) + 1);   // high byte
    buf[4] = *((uint8_t*)(&angle) + 0);         // low byte
    buf[5] = *((uint8_t*)(&angle) + 1);
    buf[6] = *((uint8_t*)(&angle) + 2);
    buf[7] = *((uint8_t*)(&angle) + 3);         // high byte
}

void MG8016EI6::create_cmd_angle_increment_control(uint8_t buf[8], int32_t angle_increment) {
    buf[0] = CMD_ANGLE_INCREMENT_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&angle_increment) + 0);   // low byte
    buf[5] = *((uint8_t*)(&angle_increment) + 1);
    buf[6] = *((uint8_t*)(&angle_increment) + 2);
    buf[7] = *((uint8_t*)(&angle_increment) + 3);   // high byte   
}

void MG8016EI6::create_cmd_angle_increment_control_speed_limited(uint8_t buf[8], int32_t angle_increment, uint16_t speed_limit) {
    buf[0] = CMD_ANGLE_INCREMENT_CONTROL_SPEED_LIMITED;
    buf[1] = 0;
    buf[2] = *((uint8_t*)(&speed_limit) + 0);       // low byte
    buf[3] = *((uint8_t*)(&speed_limit) + 1);       // high byte
    buf[4] = *((uint8_t*)(&angle_increment) + 0);   // low byte
    buf[5] = *((uint8_t*)(&angle_increment) + 1);
    buf[6] = *((uint8_t*)(&angle_increment) + 2);
    buf[7] = *((uint8_t*)(&angle_increment) + 3);   // high byte
}

void MG8016EI6::create_cmd_read_pid(uint8_t buf[8]) {
    buf[0] = CMD_READ_PID;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_write_pid(uint8_t buf[8], uint8_t angle_p, uint8_t angle_i, uint8_t speed_p, uint8_t speed_i, uint8_t torque_p, uint8_t torque_i) {
    buf[0] = CMD_WRITE_PID;
    buf[1] = 0;
    buf[2] = angle_p;
    buf[3] = angle_i;
    buf[4] = speed_p;
    buf[5] = speed_i;
    buf[6] = torque_p;
    buf[7] = torque_i;
}

void MG8016EI6::create_cmd_write_pid_rom(uint8_t buf[8], uint8_t angle_p, uint8_t angle_i, uint8_t speed_p, uint8_t speed_i, uint8_t torque_p, uint8_t torque_i) {
    buf[0] = CMD_WRITE_PID_ROM;
    buf[1] = 0;
    buf[2] = angle_p;
    buf[3] = angle_i;
    buf[4] = speed_p;
    buf[5] = speed_i;
    buf[6] = torque_p;
    buf[7] = torque_i;
}

void MG8016EI6::create_cmd_read_acceleration(uint8_t buf[8]) {
    buf[0] = CMD_READ_ACCELERATION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_write_acceleration(uint8_t buf[8], int32_t acceleration) {
    buf[0] = CMD_WRITE_ACCELERATION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&acceleration) + 0);   // low byte
    buf[5] = *((uint8_t*)(&acceleration) + 1);
    buf[6] = *((uint8_t*)(&acceleration) + 2);
    buf[7] = *((uint8_t*)(&acceleration) + 3);   // high byte
}

void MG8016EI6::create_cmd_read_encoder(uint8_t buf[8]) {
    buf[0] = CMD_READ_ENCODER;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_write_encoder_zero(uint8_t buf[8], uint16_t encoder_offset) {
    buf[0] = CMD_WRITE_ENCODER_ZERO;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = *((uint8_t*)(&encoder_offset) + 0);   // low byte
    buf[7] = *((uint8_t*)(&encoder_offset) + 1);   // high byte
}

void MG8016EI6::create_cmd_write_position_as_zero(uint8_t buf[8]) {
    buf[0] = CMD_WRITE_POSITION_AS_ZERO;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_read_multi_angle(uint8_t buf[8]) {
    buf[0] = CMD_READ_MULTI_ANGLE;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_read_angle(uint8_t buf[8]) {
    buf[0] = CMD_READ_ANGLE;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_read_state_1(uint8_t buf[8]) {
    buf[0] = CMD_READ_STATE_1;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_clear_error(uint8_t buf[8]) {
    buf[0] = CMD_CLEAR_ERROR;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_read_state_2(uint8_t buf[8]) {
    buf[0] = CMD_READ_STATE_2;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void MG8016EI6::create_cmd_read_state_3(uint8_t buf[8]) {
    buf[0] = CMD_READ_STATE_3;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}