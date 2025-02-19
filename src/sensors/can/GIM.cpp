#include "GIM.hpp"

void GIM::init() {
    write_motor_on();
}

int GIM::read(CAN_message_t& msg) {
    // check the msg id and bus to see if this msg is for this motor
    if (!check_msg_id(msg)) return 0;

    uint8_t cmd_byte = msg.buf[0];

    switch (cmd_byte) {
    // commands that return temperature, position, speed, torque
    case CMD_TORQUE_CONTROL:
    case CMD_SPEED_CONTROL: {
        // Byte2 is temperature
        m_state.temperature = msg.buf[2];

        // Byte3–4 (Pos0, Pos1) form a 16-bit position: pos_int = (Pos1 << 8) | Pos0
        uint16_t pos_int = (msg.buf[4] << 8) | msg.buf[3];
        float pos_float = (float)pos_int * 25.0f / 65535.0f - 12.5f;
        m_state.position = pos_float; // Keep it as float, per spec

        // Speed: 12 bits = ST0 (byte 5) as high 8 bits, ST1[7:4] (top nibble) as low 4 bits
        uint16_t speed_int = ((uint16_t)msg.buf[5] << 4) | (msg.buf[6] >> 4);
        float speed_float = (float)speed_int * 130.0f / 4095.0f - 65.0f;
        m_state.speed = speed_float;

        // Torque: 12 bits = ST1[3:0] (low nibble) as high 4 bits, ST2 (byte 7) as low 8 bits
        uint16_t torque_int = ((uint16_t)(msg.buf[6] & 0x0F) << 8) | msg.buf[7];
        float torque_float =
            (float)torque_int * (450.0f * m_torque_constant * m_gear_ratio) / 4095.0f
            - (225.0f * m_torque_constant * m_gear_ratio);
        m_state.torque = torque_float;

    }
    // commands that don't return anything
    case CMD_STOP_MOTOR:
    case CMD_START_MOTOR:
    case CMD_STOP_CONTROL: {
        break;
    }
    default:
        Serial.printf("No GIM::read case for this command byte: 0x%02X\n", cmd_byte);
        break;
    }

    return 1;
}

int GIM::write(CAN_message_t& msg) const {
    memcpy(&msg, &m_output, sizeof(CAN_message_t));
    return 0;
}

void GIM::zero_motor() {
    // stop the motor
    write_motor_torque(0.f);
}

void GIM::write_motor_on() {
    uint8_t buf[8];
    create_cmd_start_motor(buf);

    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void GIM::write_motor_off() {
    uint8_t buf[8];
    create_cmd_stop_motor(buf);

    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void GIM::write_motor_torque(float torque) {
    if (torque < -1.0f) torque = -1.0f;
    if (torque > 1.0f) torque = 1.0f;

    // map the -1f to 1f torque value to the actual torque value for this specific motor
    float mapped_torque = torque * m_max_torque;

    // create the command
    uint8_t buf[8];
    create_cmd_torque_control(buf, mapped_torque, 0);

    // fill in the output array
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void GIM::write_motor_speed(float speed) {
    // clamp speed argument from -1.0f to 1.0f
    if (speed < -1.0f) speed = -1.0f;
    if (speed > 1.0f) speed = 1.0f;

    // map speed to the motor's max speed
    float mapped_rpm = speed * m_max_speed;

    // create the command
    uint8_t buf[8];
    create_cmd_speed_control(buf, mapped_rpm, 0); // hardcore 0 duration until we find out what it does

    // fill the output
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void GIM::write_motor_position(float position) {
    // create the command
    uint8_t buf[8];
    create_cmd_position_control(buf, position, 0); // hardcore 0 duration until we find out what it does

    // fill the output
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}

void GIM::write_motor_stop() {
    uint8_t buf[8];
    create_cmd_stop_control(buf);

    // fill the output
    m_output.id = m_base_id + m_id;
    for (int i = 0; i < 8; i++) {
        m_output.buf[i] = buf[i];
    }
}


// command create functions

void GIM::create_cmd_reset_configuration(uint8_t buf[8]) {
    buf[0] = CMD_RESET_CONFIGURATION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_refresh_configuration(uint8_t buf[8]) {
    buf[0] = CMD_REFRESH_CONFIGURATION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_modify_configuration(uint8_t buf[8], uint8_t conf_type, uint8_t conf_id, uint32_t data) {
    buf[0] = CMD_MODIFY_CONFIGURATION;
    buf[1] = conf_type;
    buf[2] = conf_id;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&data) + 0); // low byte
    buf[5] = *((uint8_t*)(&data) + 1);
    buf[6] = *((uint8_t*)(&data) + 2);
    buf[7] = *((uint8_t*)(&data) + 3); // high byte
}

void GIM::create_cmd_retrieve_configuration(uint8_t buf[8], uint8_t conf_type, uint8_t conf_id) {
    buf[0] = CMD_RETRIEVE_CONFIGURATION;
    buf[1] = conf_type;
    buf[2] = conf_id;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_start_motor(uint8_t buf[8]) {
    buf[0] = CMD_START_MOTOR;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_stop_motor(uint8_t buf[8]) {
    buf[0] = CMD_STOP_MOTOR;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_torque_control(uint8_t buf[8], float torque, uint32_t duration) {
    buf[0] = CMD_TORQUE_CONTROL;
    buf[1] = *((uint8_t*)(&torque) + 0); // low byte
    buf[2] = *((uint8_t*)(&torque) + 1);
    buf[3] = *((uint8_t*)(&torque) + 2);
    buf[4] = *((uint8_t*)(&torque) + 3); // high byte
    buf[5] = *((uint8_t*)(&duration) + 0); // low byte
    buf[6] = *((uint8_t*)(&duration) + 1);
    buf[7] = *((uint8_t*)(&duration) + 2); // high byte
}

void GIM::create_cmd_speed_control(uint8_t buf[8], float speed, uint32_t duration) {
    buf[0] = CMD_SPEED_CONTROL;
    buf[1] = *((uint8_t*)(&speed) + 0); // low byte
    buf[2] = *((uint8_t*)(&speed) + 1);
    buf[3] = *((uint8_t*)(&speed) + 2);
    buf[4] = *((uint8_t*)(&speed) + 3); // high byte
    buf[5] = *((uint8_t*)(&duration) + 0); // low byte
    buf[6] = *((uint8_t*)(&duration) + 1);
    buf[7] = *((uint8_t*)(&duration) + 2); // high byte
}


void GIM::create_cmd_position_control(uint8_t buf[8], float position, uint32_t duration) {
    buf[0] = CMD_POSITION_CONTROL;
    buf[1] = *((uint8_t*)(&position) + 0); // low byte
    buf[2] = *((uint8_t*)(&position) + 1);
    buf[3] = *((uint8_t*)(&position) + 2);
    buf[4] = *((uint8_t*)(&position) + 3); // high byte
    buf[5] = *((uint8_t*)(&duration) + 0); // low byte
    buf[6] = *((uint8_t*)(&duration) + 1);
    buf[7] = *((uint8_t*)(&duration) + 2); // high byte
}


void GIM::create_cmd_stop_control(uint8_t buf[8]) {
    buf[0] = CMD_STOP_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_modify_parameter(uint8_t buf[8], uint8_t param_id, uint32_t data) {
    buf[0] = CMD_MODIFY_PARAMETER;
    buf[1] = param_id;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&data) + 0); // low byte
    buf[5] = *((uint8_t*)(&data) + 1);
    buf[6] = *((uint8_t*)(&data) + 2);
    buf[7] = *((uint8_t*)(&data) + 3); // high byte
}

void GIM::create_cmd_retrieve_parameter(uint8_t buf[8], uint8_t param_id) {
    buf[0] = CMD_RETRIEVE_PARAMETER;
    buf[1] = param_id;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_get_version(uint8_t buf[8]) {
    buf[0] = CMD_GET_VERSION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_get_fault(uint8_t buf[8]) {
    buf[0] = CMD_GET_FAULT;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_acknowledge_fault(uint8_t buf[8]) {
    buf[0] = CMD_ACKNOWLEDGE_FAULT;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_retrieve_indicator(uint8_t buf[8], uint8_t indicator_id) {
    buf[0] = CMD_RETRIEVE_INDICATOR;
    buf[1] = indicator_id;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_calibration(uint8_t buf[8], uint8_t calibration_type) {
    buf[0] = CMD_CALIBRATION;
    buf[1] = calibration_type;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void GIM::create_cmd_update_firmware(uint8_t buf[8], uint8_t indicator_id) {
    buf[0] = CMD_UPDATE_FIRMWARE;
    buf[1] = indicator_id;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}
