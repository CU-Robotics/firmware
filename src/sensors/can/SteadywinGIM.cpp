#include "SteadywinGIM.hpp"
#include "utils/system_log.hpp"

void SteadywinGIM::init() {
    write_motor_on();
}

int SteadywinGIM::read(CAN_message_t& msg) {
    // check the msg id and bus to see if this msg is for this motor
    if (!check_msg_id(msg)) return 0;

    uint8_t cmd_byte = msg.buf[0];

    // Byte1 of every reply is the result code, 0x00 is success
    uint8_t res = msg.buf[1];
    if (res != 0x00) {
        // mark unhealthy so commands are zeroed and the start command is retried. Rate limit the log, the motor
        // replies to every command
        m_rejected = true;
        m_rejects_since_log++;
        if (millis() - m_last_reject_log_ms >= REJECT_LOG_INTERVAL_MS) {
            SystemLog.error(Subsystem::MOTORS, "GIM %u: command 0x%02X failed with RES 0x%02X (%lu since last log)\n", m_physical_id, cmd_byte, res, m_rejects_since_log);
            m_last_reject_log_ms = millis();
            m_rejects_since_log = 0;
        }
        return 1;
    }

    switch (cmd_byte) {
    // commands that return temperature, position, speed, torque
    case CMD_TORQUE_CONTROL:
    case CMD_SPEED_CONTROL: {
        // Byte2 is temperature
        m_state.temperature = (int8_t)msg.buf[2];

        // Byte3–4 (Pos0, Pos1) form a 16-bit position: pos_int = (Pos1 << 8) | Pos0
        uint16_t pos_int = (msg.buf[4] << 8) | msg.buf[3];
        m_state.position = pos_int;
        // unwrap: the motor moves far less than half the 16 bit range between replies, so the shortest signed step
        // between consecutive readings is the real one
        if (m_have_status)
            m_position_counts += (int16_t)(uint16_t)(pos_int - m_last_position_raw);
        else
            m_position_counts = pos_int;
        m_last_position_raw = pos_int;
        m_position_rad = (float)m_position_counts * 25.0f / 65535.0f - 12.5f;

        // Speed: 12 bits = ST0 (byte 5) as high 8 bits, ST1[7:4] (top nibble) as low 4 bits. rad/s at the output
        uint16_t speed_int = ((uint16_t)msg.buf[5] << 4) | (msg.buf[6] >> 4);
        m_state.speed = (float)speed_int * 130.0f / 4095.0f - 65.0f;

        // Torque: 12 bits = ST1[3:0] (low nibble) as high 4 bits, ST2 (byte 7) as low 8 bits
        // The doc's formula is torque_int * (450 * Kt * GR) / 4095 - 225 * Kt * GR with Kt the motor-side
        // torque constant. m_torque_constant is output-side (already Kt * GR), so don't multiply by m_gear_ratio again
        uint16_t torque_int = ((uint16_t)(msg.buf[6] & 0x0F) << 8) | msg.buf[7];
        float torque_nm = (float)torque_int * (450.0f * m_torque_constant) / 4095.0f - (225.0f * m_torque_constant);
        m_state.torque = torque_nm / m_max_torque;

        m_have_status = true;
        m_last_status_ms = millis();
        m_rejected = false;
        break;
    }
    case CMD_START_MOTOR: {
        // started, torque/speed commands should be accepted again
        m_rejected = false;
        break;
    }
    // commands that don't return anything
    case CMD_STOP_MOTOR:
    case CMD_STOP_CONTROL: {
        break;
    }
    default:
        SystemLog.error(Subsystem::MOTORS,"No SteadywinGIM::read case for this command byte: 0x%02X\n", cmd_byte);
        break;
    }

    return 1;
}

int SteadywinGIM::write(CAN_message_t& msg) const {
    memcpy(&msg, &m_output, sizeof(CAN_message_t));
    m_start_pending = false;
    return 0;
}

bool SteadywinGIM::is_healthy() const {
    return m_have_status && !m_rejected && millis() - m_last_status_ms < STALE_TIMEOUT_MS;
}

bool SteadywinGIM::prepare_command() {
    // a start command waiting to go out takes priority
    if (m_start_pending)
        return false;

    // the motor rejects commands until it's started, and a motor powered after init never got the start from init.
    // Resend it every so often while unhealthy
    if (!is_healthy() && millis() - m_start_requested_ms > RESTART_RETRY_MS) {
        write_motor_on();
        return false;
    }
    return true;
}

void SteadywinGIM::zero_motor() {
    // stop the motor
    execute_motor_torque_command(0.0f);
}

void SteadywinGIM::set_output(const uint8_t buf[8]) {
    m_output.id = m_base_id + m_physical_id;
    m_output.len = 8;
    memcpy(m_output.buf, buf, 8);
}

void SteadywinGIM::write_motor_on() {
    uint8_t buf[8];
    create_cmd_start_motor(buf);
    set_output(buf);
    m_start_pending = true;
    m_start_requested_ms = millis();
}

void SteadywinGIM::write_motor_off() {
    uint8_t buf[8];
    create_cmd_stop_motor(buf);
    set_output(buf);
}


void SteadywinGIM::execute_motor_torque_command(float torque) {
    if (torque < -1.0f) torque = -1.0f;
    if (torque > 1.0f) torque = 1.0f;

    // map the -1f to 1f torque value to the actual torque value for this specific motor
    float mapped_torque = torque * m_max_torque;

    // the motor's driver ignores commands above rated current * Kt instead of clipping them, so clip here
    float limit = m_rated_current * m_torque_constant * TORQUE_LIMIT_MARGIN;
    if (mapped_torque > limit) mapped_torque = limit;
    if (mapped_torque < -limit) mapped_torque = -limit;

    if (!prepare_command())
        return;
    // don't drive on stale or rejected state. Still send zero torque: the reply is how the motor's state recovers
    if (!is_healthy())
        mapped_torque = 0.0f;

    uint8_t buf[8];
    create_cmd_torque_control(buf, mapped_torque);
    set_output(buf);
}

void SteadywinGIM::write_motor_speed(float speed) {
    // clamp speed argument from -1.0f to 1.0f
    if (speed < -1.0f) speed = -1.0f;
    if (speed > 1.0f) speed = 1.0f;

    // map speed to the motor's max speed
    float mapped_rpm = speed * m_max_speed;

    if (!prepare_command())
        return;
    if (!is_healthy())
        mapped_rpm = 0.0f;

    uint8_t buf[8];
    create_cmd_speed_control(buf, mapped_rpm);
    set_output(buf);
}

void SteadywinGIM::write_motor_stop() {
    uint8_t buf[8];
    create_cmd_stop_control(buf);
    set_output(buf);
}


// command create functions

void SteadywinGIM::create_cmd_reset_configuration(uint8_t buf[8]) {
    buf[0] = CMD_RESET_CONFIGURATION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_refresh_configuration(uint8_t buf[8]) {
    buf[0] = CMD_REFRESH_CONFIGURATION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_modify_configuration(uint8_t buf[8], uint8_t conf_type, uint8_t conf_id, uint32_t data) {
    buf[0] = CMD_MODIFY_CONFIGURATION;
    buf[1] = conf_type;
    buf[2] = conf_id;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&data) + 0); // low byte
    buf[5] = *((uint8_t*)(&data) + 1);
    buf[6] = *((uint8_t*)(&data) + 2);
    buf[7] = *((uint8_t*)(&data) + 3); // high byte
}

void SteadywinGIM::create_cmd_retrieve_configuration(uint8_t buf[8], uint8_t conf_type, uint8_t conf_id) {
    buf[0] = CMD_RETRIEVE_CONFIGURATION;
    buf[1] = conf_type;
    buf[2] = conf_id;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_start_motor(uint8_t buf[8]) {
    buf[0] = CMD_START_MOTOR;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_stop_motor(uint8_t buf[8]) {
    buf[0] = CMD_STOP_MOTOR;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

// Bytes 5-7 of torque and speed commands are an execution time in ms. 0 holds the command until the next one,
// which is what we want since the controller sends one every loop. It isn't used as a command timeout: on position
// control it acts as the move duration, so a non-zero value here would likely ramp torque instead of timing out.
// Stopping the motor when the Teensy goes quiet has to come from the motor's own config, if it supports one

void SteadywinGIM::create_cmd_torque_control(uint8_t buf[8], float torque) {
    buf[0] = CMD_TORQUE_CONTROL;
    buf[1] = *((uint8_t*)(&torque) + 0); // low byte
    buf[2] = *((uint8_t*)(&torque) + 1);
    buf[3] = *((uint8_t*)(&torque) + 2);
    buf[4] = *((uint8_t*)(&torque) + 3); // high byte
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_speed_control(uint8_t buf[8], float speed) {
    buf[0] = CMD_SPEED_CONTROL;
    buf[1] = *((uint8_t*)(&speed) + 0); // low byte
    buf[2] = *((uint8_t*)(&speed) + 1);
    buf[3] = *((uint8_t*)(&speed) + 2);
    buf[4] = *((uint8_t*)(&speed) + 3); // high byte
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}


void SteadywinGIM::create_cmd_stop_control(uint8_t buf[8]) {
    buf[0] = CMD_STOP_CONTROL;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_modify_parameter(uint8_t buf[8], uint8_t param_id, uint32_t data) {
    buf[0] = CMD_MODIFY_PARAMETER;
    buf[1] = param_id;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = *((uint8_t*)(&data) + 0); // low byte
    buf[5] = *((uint8_t*)(&data) + 1);
    buf[6] = *((uint8_t*)(&data) + 2);
    buf[7] = *((uint8_t*)(&data) + 3); // high byte
}

void SteadywinGIM::create_cmd_retrieve_parameter(uint8_t buf[8], uint8_t param_id) {
    buf[0] = CMD_RETRIEVE_PARAMETER;
    buf[1] = param_id;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_get_version(uint8_t buf[8]) {
    buf[0] = CMD_GET_VERSION;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_get_fault(uint8_t buf[8]) {
    buf[0] = CMD_GET_FAULT;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_acknowledge_fault(uint8_t buf[8]) {
    buf[0] = CMD_ACKNOWLEDGE_FAULT;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_retrieve_indicator(uint8_t buf[8], uint8_t indicator_id) {
    buf[0] = CMD_RETRIEVE_INDICATOR;
    buf[1] = indicator_id;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_calibration(uint8_t buf[8], uint8_t calibration_type) {
    buf[0] = CMD_CALIBRATION;
    buf[1] = calibration_type;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

void SteadywinGIM::create_cmd_update_firmware(uint8_t buf[8], uint8_t indicator_id) {
    buf[0] = CMD_UPDATE_FIRMWARE;
    buf[1] = indicator_id;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}
