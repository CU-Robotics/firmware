#include "MG8016EI6.hpp"

template <typename CAN_BUS>
int MG8016EI6<CAN_BUS>::read(CAN_message_t& msg) {
    // command IDs that do not return state data:
    return 1;
}

template<typename CAN_BUS>
int MG8016EI6<CAN_BUS>::write(CAN_message_t& msg) {
    // TODO: does the caller need my local ID?
    
    memcpy(&msg, &m_output, sizeof(CAN_message_t));

    return 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::write_motor_torque(float torque) {
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

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::write_motor_speed(float speed) {
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

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::write_motor_angle(float angle, float speed_limit) {
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

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::print_state() {
    Serial.printf("Unimplemented\n");
}

// Command creation functions

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_motor_off(uint8_t buf[8]) {
    buf[0] = CMD_MOTOR_OFF;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_motor_on(uint8_t buf[8]) {
    buf[0] = CMD_MOTOR_ON;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_motor_stop(uint8_t buf[8]) {
    buf[0] = CMD_MOTOR_STOP;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_torque_control(uint8_t buf[8], int16_t torque) {
    buf[0] = CMD_TORQUE_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&torque) + 0);         // low byte
    buf[5] = *((uint8_t*)(&torque) + 1);         // high byte
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_speed_control(uint8_t buf[8], int32_t speed) {
    buf[0] = CMD_SPEED_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&speed) + 0);         // low byte
    buf[5] = *((uint8_t*)(&speed) + 1);
    buf[6] = *((uint8_t*)(&speed) + 2);
    buf[7] = *((uint8_t*)(&speed) + 3);         // high byte
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_multi_angle_control(uint8_t buf[8], int32_t angle) {
    buf[0] = CMD_MULTI_ANGLE_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&angle) + 0);         // low byte
    buf[5] = *((uint8_t*)(&angle) + 1);
    buf[6] = *((uint8_t*)(&angle) + 2);
    buf[7] = *((uint8_t*)(&angle) + 3);         // high byte
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_multi_angle_control_speed_limited(uint8_t buf[8], int32_t angle, uint16_t speed_limit) {
    buf[0] = CMD_MULTI_ANGLE_CONTROL_SPEED_LIMITED;
    buf[1] = 0;
    buf[2] = *((uint8_t*)(&speed_limit) + 0);   // low byte
    buf[3] = *((uint8_t*)(&speed_limit) + 1);   // high byte
    buf[4] = *((uint8_t*)(&angle) + 0);         // low byte
    buf[5] = *((uint8_t*)(&angle) + 1);
    buf[6] = *((uint8_t*)(&angle) + 2);
    buf[7] = *((uint8_t*)(&angle) + 3);         // high byte
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_angle_control(uint8_t buf[8], uint32_t angle, uint8_t spin_direction) {
    buf[0] = CMD_ANGLE_CONTROL;
    buf[1] = spin_direction;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&angle) + 0);         // low byte
    buf[5] = *((uint8_t*)(&angle) + 1);
    buf[6] = *((uint8_t*)(&angle) + 2);
    buf[7] = *((uint8_t*)(&angle) + 3);         // high byte
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_angle_control_speed_limited(uint8_t buf[8], uint32_t angle, uint8_t spin_direction, uint16_t speed_limit) {
    buf[0] = CMD_ANGLE_CONTROL_SPEED_LIMITED;
    buf[1] = spin_direction;
    buf[2] = *((uint8_t*)(&speed_limit) + 0);   // low byte
    buf[3] = *((uint8_t*)(&speed_limit) + 1);   // high byte
    buf[4] = *((uint8_t*)(&angle) + 0);         // low byte
    buf[5] = *((uint8_t*)(&angle) + 1);
    buf[6] = *((uint8_t*)(&angle) + 2);
    buf[7] = *((uint8_t*)(&angle) + 3);         // high byte
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_angle_increment_control(uint8_t buf[8], int32_t angle_increment) {
    buf[0] = CMD_ANGLE_INCREMENT_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&angle_increment) + 0);   // low byte
    buf[5] = *((uint8_t*)(&angle_increment) + 1);
    buf[6] = *((uint8_t*)(&angle_increment) + 2);
    buf[7] = *((uint8_t*)(&angle_increment) + 3);   // high byte   
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_angle_increment_control_speed_limited(uint8_t buf[8], int32_t angle_increment, uint16_t speed_limit) {
    buf[0] = CMD_ANGLE_INCREMENT_CONTROL_SPEED_LIMITED;
    buf[1] = 0;
    buf[2] = *((uint8_t*)(&speed_limit) + 0);       // low byte
    buf[3] = *((uint8_t*)(&speed_limit) + 1);       // high byte
    buf[4] = *((uint8_t*)(&angle_increment) + 0);   // low byte
    buf[5] = *((uint8_t*)(&angle_increment) + 1);
    buf[6] = *((uint8_t*)(&angle_increment) + 2);
    buf[7] = *((uint8_t*)(&angle_increment) + 3);   // high byte
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_read_pid(uint8_t buf[8]) {
    buf[0] = CMD_READ_PID;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_write_pid(uint8_t buf[8], uint8_t angle_p, uint8_t angle_i, uint8_t speed_p, uint8_t speed_i, uint8_t torque_p, uint8_t torque_i) {
    buf[0] = CMD_WRITE_PID;
    buf[1] = 0;
    buf[2] = angle_p;
    buf[3] = angle_i;
    buf[4] = speed_p;
    buf[5] = speed_i;
    buf[6] = torque_p;
    buf[7] = torque_i;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_write_pid_rom(uint8_t buf[8], uint8_t angle_p, uint8_t angle_i, uint8_t speed_p, uint8_t speed_i, uint8_t torque_p, uint8_t torque_i) {
    buf[0] = CMD_WRITE_PID_ROM;
    buf[1] = 0;
    buf[2] = angle_p;
    buf[3] = angle_i;
    buf[4] = speed_p;
    buf[5] = speed_i;
    buf[6] = torque_p;
    buf[7] = torque_i;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_read_acceleration(uint8_t buf[8]) {
    buf[0] = CMD_READ_ACCELERATION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_write_acceleration(uint8_t buf[8], int32_t acceleration) {
    buf[0] = CMD_WRITE_ACCELERATION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&acceleration) + 0);   // low byte
    buf[5] = *((uint8_t*)(&acceleration) + 1);
    buf[6] = *((uint8_t*)(&acceleration) + 2);
    buf[7] = *((uint8_t*)(&acceleration) + 3);   // high byte
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_read_encoder(uint8_t buf[8]) {
    buf[0] = CMD_READ_ENCODER;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_write_encoder_zero(uint8_t buf[8], uint16_t encoder_offset) {
    buf[0] = CMD_WRITE_ENCODER_ZERO;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = *((uint8_t*)(&encoder_offset) + 0);   // low byte
    buf[7] = *((uint8_t*)(&encoder_offset) + 1);   // high byte
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_write_position_as_zero(uint8_t buf[8]) {
    buf[0] = CMD_WRITE_POSITION_AS_ZERO;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_read_multi_angle(uint8_t buf[8]) {
    buf[0] = CMD_READ_MULTI_ANGLE;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_read_angle(uint8_t buf[8]) {
    buf[0] = CMD_READ_ANGLE;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_read_state_1(uint8_t buf[8]) {
    buf[0] = CMD_READ_STATE_1;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_clear_error(uint8_t buf[8]) {
    buf[0] = CMD_CLEAR_ERROR;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_read_state_2(uint8_t buf[8]) {
    buf[0] = CMD_READ_STATE_2;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

template<typename CAN_BUS>
void MG8016EI6<CAN_BUS>::create_cmd_read_state_3(uint8_t buf[8]) {
    buf[0] = CMD_READ_STATE_3;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

// explicitly declare the three possible types C610 can take

template class MG8016EI6<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>>;
template class MG8016EI6<FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>>;
template class MG8016EI6<FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>>;