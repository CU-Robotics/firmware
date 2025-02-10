#include "SDC104.hpp"

void SDC104::init() {
    // axis state is defered until the first control command is sent
    // set_axis_state(AxisState::CLOSED_LOOP_CONTROL);
    set_controller_mode(ControlMode::TORQUE, InputMode::DIRECT);
}

int SDC104::read(CAN_message_t& msg) {
    // check if the message is for this motor
    if (!check_msg_id(msg)) return 0;

    // cmd byte is the first 5 bits
    uint8_t cmd_byte = msg.id & 0x1F;

    switch (cmd_byte) {
    case CMD_HEARTBEAT: {
        // copy the data into the heartbeat struct direct
        memcpy(&m_heartbeat, msg.buf, 8);
        break;
    }
    case CMD_GET_ERROR: {
        switch (m_error_request_type) {
        case ErrorRequestType::MOTOR: {
            m_motor_exception = *((uint64_t*)&msg.buf[0]);
            break;
        }
        case ErrorRequestType::ENCODER: {
            m_encoder_exception = *((uint32_t*)&msg.buf[0]);
            break;
        }
        case ErrorRequestType::CONTROL: {
            m_control_exception = *((uint32_t*)&msg.buf[0]);
            break;
        }
        case ErrorRequestType::SYSTEM: {
            m_system_exception = *((uint32_t*)&msg.buf[0]);
            break;
        }
        default: {
            Serial.printf("Unknown error type: %d\n", static_cast<int>(m_error_request_type));
            break;
        }
        }
        break;
    }
    case CMD_MIT_CONTROL: { Serial.printf("MIT control command not implemented\n"); break; }
    case CMD_GET_ENCODER_ESTIMATES: {
        m_position_estimate = *((float*)&msg.buf[0]);
        m_velocity_estimate = *((float*)&msg.buf[4]);

        // update the unified state struct
        // encoder gives revolutions, we want radians
        m_state.position = m_position_estimate * 2 * PI;
        m_state.speed = m_velocity_estimate * 2 * PI;
        break;
    }
    case CMD_GET_ENCODER_COUNT: {
        m_encoder_shadow_count = *((int32_t*)&msg.buf[0]);
        m_encoder_revolution_count = *((int32_t*)&msg.buf[4]);
        break;
    }
    case CMD_GET_IQ: {
        m_iq_setpoint = *((float*)&msg.buf[0]);
        m_iq_measured = *((float*)&msg.buf[4]);
        break;
    }
    case CMD_GET_BUS_VOLTAGE_CURRENT: {
        m_bus_voltage = *((float*)&msg.buf[0]);
        m_bus_current = *((float*)&msg.buf[4]);

        // update the unified state struct
        float torque = m_bus_current * m_torque_constant;
        // normalize torque to -1 to 1
        torque /= m_max_torque;

        m_state.torque = torque;
        break;
    }
    case CMD_GET_TORQUES: {
        m_torque_setpoint = *((float*)&msg.buf[0]);
        m_torque_measured = *((float*)&msg.buf[4]);

        // update the unified state struct
        m_state.torque = m_torque_measured / m_max_torque;
        break;
    }
    case CMD_GET_POWERS: {
        m_electrical_power = *((float*)&msg.buf[0]);
        m_mechanical_power = *((float*)&msg.buf[4]);
        break;
    }
    default: {
        Serial.printf("Unknown command byte: %d\n", cmd_byte);
        break;
    }
    }

    return 1;
}

int SDC104::write(CAN_message_t& msg) const {
    memcpy(&msg, &m_output, sizeof(CAN_message_t));

    return 0;
}

void SDC104::zero_motor() {
    write_motor_torque(0.0f);
}

void SDC104::write_motor_torque(float torque) {
    // clamp the torque value
    if (torque < -1.0f) torque = -1.0f;
    if (torque > 1.0f) torque = 1.0f;

    // map the -1f to 1f torque value to the actual torque value for this specific motor
    float mapped_torque = torque * m_max_torque;

    // if the motor isn't in closed loop control, set it to closed loop control instead
    // the motor wont accept torque commands in any other state
    if (m_heartbeat.axis_state != AxisState::CLOSED_LOOP_CONTROL) {
        set_axis_state(AxisState::CLOSED_LOOP_CONTROL);
        return;
    }

    // create the command
    CAN_message_t msg;
    create_cmd_set_input_torque(msg, mapped_torque);

    // fill in the output array
    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::write_motor_speed(float speed) {
    // if the motor isn't in closed loop control, set it to closed loop control instead
    // the motor wont accept speed commands in any other state
    if (m_heartbeat.axis_state != AxisState::CLOSED_LOOP_CONTROL) {
        set_axis_state(AxisState::CLOSED_LOOP_CONTROL);
        return;
    }

    // create the command
    CAN_message_t msg;
    create_cmd_set_input_velocity(msg, speed, 0);

    // fill in the output array
    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::print_state() const {
    Motor::print_state();
    Serial.printf("Axis state: %d\n", static_cast<int>(m_heartbeat.axis_state));
}

void SDC104::get_error(uint8_t error_type) {
    CAN_message_t msg;
    create_cmd_get_error(msg, error_type);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::set_axis_state(AxisState axis_state) {
    CAN_message_t msg;
    create_cmd_set_axis_state(msg, axis_state);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::get_encoder_count() {
    CAN_message_t msg;
    create_cmd_get_encoder_count(msg);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::set_limits(float velocity_limit, float current_limit) {
    CAN_message_t msg;
    create_cmd_set_limits(msg, velocity_limit, current_limit);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::set_controller_mode(ControlMode control_mode, InputMode input_mode) {
    CAN_message_t msg;
    create_cmd_set_controller_mode(msg, static_cast<uint32_t>(control_mode), static_cast<uint32_t>(input_mode));

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::get_bus_voltage_current() {
    CAN_message_t msg;
    create_cmd_get_bus_voltage_current(msg);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::clear_errors() {
    CAN_message_t msg;
    create_cmd_clear_errors(msg);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::set_linear_count(int32_t linear_count) {
    CAN_message_t msg;
    create_cmd_set_linear_count(msg, linear_count);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::set_position_gains(float pos_gain) {
    CAN_message_t msg;
    create_cmd_set_position_gains(msg, pos_gain);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::set_velocity_gains(float vel_gain, float vel_integrator_gain) {
    CAN_message_t msg;
    create_cmd_set_velocity_gains(msg, vel_gain, vel_integrator_gain);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::save_configuration() {
    CAN_message_t msg;
    create_cmd_save_configuration(msg);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

void SDC104::reboot() {
    CAN_message_t msg;
    create_cmd_reboot(msg);

    memcpy(&m_output, &msg, sizeof(CAN_message_t));
}

bool SDC104::check_msg_id(const CAN_message_t& msg) const {
    // last 6 bits of the ID are the motor ID
    uint8_t id = (msg.id >> 5) & 0x3F;

    // check if the ID is correct
    if (id != m_base_id + m_id) {
        return false;
    }

    // check if the bus is correct
    if ((uint32_t)(msg.bus - 1) != m_bus_id) {
        return false;
    }

    return true;
}

void SDC104::create_cmd_estop(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_ESTOP;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_get_error(CAN_message_t& msg, uint8_t error_type) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_GET_ERROR;
    memset(msg.buf, 0, 8);

    msg.buf[0] = error_type;

    // set the error request type
    m_error_request_type = static_cast<ErrorRequestType>(error_type);
}

void SDC104::create_cmd_set_axis_node_id(CAN_message_t& msg, uint32_t node_id) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_AXIS_NODE_ID;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&node_id + 0);
    msg.buf[1] = *((uint8_t*)&node_id + 1);
    msg.buf[2] = *((uint8_t*)&node_id + 2);
    msg.buf[3] = *((uint8_t*)&node_id + 3);
}

void SDC104::create_cmd_set_axis_state(CAN_message_t& msg, AxisState axis_state) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_AXIS_STATE;
    memset(msg.buf, 0, 8);

    msg.buf[0] = static_cast<uint8_t>(axis_state);
}

void SDC104::create_cmd_get_encoder_estimates(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_GET_ENCODER_ESTIMATES;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_get_encoder_count(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_GET_ENCODER_COUNT;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_set_controller_mode(CAN_message_t& msg, uint32_t control_mode, uint32_t input_mode) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_CONTROLLER_MODE;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&control_mode + 0);
    msg.buf[1] = *((uint8_t*)&control_mode + 1);
    msg.buf[2] = *((uint8_t*)&control_mode + 2);
    msg.buf[3] = *((uint8_t*)&control_mode + 3);

    msg.buf[4] = *((uint8_t*)&input_mode + 0);
    msg.buf[5] = *((uint8_t*)&input_mode + 1);
    msg.buf[6] = *((uint8_t*)&input_mode + 2);
    msg.buf[7] = *((uint8_t*)&input_mode + 3);
}

void SDC104::create_cmd_set_input_position(CAN_message_t& msg, float input_position, int16_t velocity_ff, int16_t torque_ff) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_INPUT_POSITION;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&input_position + 0);
    msg.buf[1] = *((uint8_t*)&input_position + 1);
    msg.buf[2] = *((uint8_t*)&input_position + 2);
    msg.buf[3] = *((uint8_t*)&input_position + 3);

    msg.buf[4] = *((uint8_t*)&velocity_ff + 0);
    msg.buf[5] = *((uint8_t*)&velocity_ff + 1);

    msg.buf[6] = *((uint8_t*)&torque_ff + 0);
    msg.buf[7] = *((uint8_t*)&torque_ff + 1);
}

void SDC104::create_cmd_set_input_velocity(CAN_message_t& msg, float input_velocity, int16_t torque_ff) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_INPUT_VELOCITY;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&input_velocity + 0);
    msg.buf[1] = *((uint8_t*)&input_velocity + 1);
    msg.buf[2] = *((uint8_t*)&input_velocity + 2);
    msg.buf[3] = *((uint8_t*)&input_velocity + 3);

    msg.buf[4] = *((uint8_t*)&torque_ff + 0);
    msg.buf[5] = *((uint8_t*)&torque_ff + 1);
}

void SDC104::create_cmd_set_input_torque(CAN_message_t& msg, float input_torque) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_INPUT_TORQUE;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&input_torque + 0);
    msg.buf[1] = *((uint8_t*)&input_torque + 1);
    msg.buf[2] = *((uint8_t*)&input_torque + 2);
    msg.buf[3] = *((uint8_t*)&input_torque + 3);
}

void SDC104::create_cmd_set_limits(CAN_message_t& msg, float velocity_limit, float current_limit) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_LIMITS;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&velocity_limit + 0);
    msg.buf[1] = *((uint8_t*)&velocity_limit + 1);
    msg.buf[2] = *((uint8_t*)&velocity_limit + 2);
    msg.buf[3] = *((uint8_t*)&velocity_limit + 3);

    msg.buf[4] = *((uint8_t*)&current_limit + 0);
    msg.buf[5] = *((uint8_t*)&current_limit + 1);
    msg.buf[6] = *((uint8_t*)&current_limit + 2);
    msg.buf[7] = *((uint8_t*)&current_limit + 3);
}

void SDC104::create_cmd_start_anticogging(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_START_ANTICOGGING;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_set_traj_velocity_limit(CAN_message_t& msg, float traj_velocity_limit) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_TRAJ_VELOCITY_LIMIT;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&traj_velocity_limit + 0);
    msg.buf[1] = *((uint8_t*)&traj_velocity_limit + 1);
    msg.buf[2] = *((uint8_t*)&traj_velocity_limit + 2);
    msg.buf[3] = *((uint8_t*)&traj_velocity_limit + 3);
}

void SDC104::create_cmd_set_traj_acceleration_limit(CAN_message_t& msg, float traj_acceleration_limit, float traj_deceleration_limit) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_TRAJ_ACCELERATION_LIMIT;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&traj_acceleration_limit + 0);
    msg.buf[1] = *((uint8_t*)&traj_acceleration_limit + 1);
    msg.buf[2] = *((uint8_t*)&traj_acceleration_limit + 2);
    msg.buf[3] = *((uint8_t*)&traj_acceleration_limit + 3);

    msg.buf[4] = *((uint8_t*)&traj_deceleration_limit + 0);
    msg.buf[5] = *((uint8_t*)&traj_deceleration_limit + 1);
    msg.buf[6] = *((uint8_t*)&traj_deceleration_limit + 2);
    msg.buf[7] = *((uint8_t*)&traj_deceleration_limit + 3);
}

void SDC104::create_cmd_set_traj_inertia(CAN_message_t& msg, float traj_inertia) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_TRAJ_INERTIA;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&traj_inertia + 0);
    msg.buf[1] = *((uint8_t*)&traj_inertia + 1);
    msg.buf[2] = *((uint8_t*)&traj_inertia + 2);
    msg.buf[3] = *((uint8_t*)&traj_inertia + 3);
}

void SDC104::create_cmd_get_iq(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_GET_IQ;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_reboot(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_REBOOT;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_get_bus_voltage_current(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_GET_BUS_VOLTAGE_CURRENT;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_clear_errors(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_CLEAR_ERRORS;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_set_linear_count(CAN_message_t& msg, int32_t linear_count) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_LINEAR_COUNT;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&linear_count + 0);
    msg.buf[1] = *((uint8_t*)&linear_count + 1);
    msg.buf[2] = *((uint8_t*)&linear_count + 2);
    msg.buf[3] = *((uint8_t*)&linear_count + 3);
}

void SDC104::create_cmd_set_position_gains(CAN_message_t& msg, float pos_gain) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_POSITION_GAINS;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&pos_gain + 0);
    msg.buf[1] = *((uint8_t*)&pos_gain + 1);
    msg.buf[2] = *((uint8_t*)&pos_gain + 2);
    msg.buf[3] = *((uint8_t*)&pos_gain + 3);
}

void SDC104::create_cmd_set_velocity_gains(CAN_message_t& msg, float vel_gain, float vel_integrator_gain) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SET_VELOCITY_GAINS;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t*)&vel_gain + 0);
    msg.buf[1] = *((uint8_t*)&vel_gain + 1);
    msg.buf[2] = *((uint8_t*)&vel_gain + 2);
    msg.buf[3] = *((uint8_t*)&vel_gain + 3);

    msg.buf[4] = *((uint8_t*)&vel_integrator_gain + 0);
    msg.buf[5] = *((uint8_t*)&vel_integrator_gain + 1);
    msg.buf[6] = *((uint8_t*)&vel_integrator_gain + 2);
    msg.buf[7] = *((uint8_t*)&vel_integrator_gain + 3);
}

void SDC104::create_cmd_get_torques(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_GET_TORQUES;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_get_powers(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_GET_POWERS;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_save_configuration(CAN_message_t& msg) {
    msg.id = ((m_base_id + m_id) << 5) | CMD_SAVE_CONFIGURATION;
    memset(msg.buf, 0, 8);
}
