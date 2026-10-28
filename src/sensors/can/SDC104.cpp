#include "SDC104.hpp"
#include "utils/system_log.hpp"

void SDC104::init() {
    // idle first: the SDC104 holds its last command forever, even across a Teensy reset. Control mode and closed
    // loop are set by the first torque/speed command
    set_axis_state(AxisState::IDLE);
}

int SDC104::read(CAN_message_t &msg) {
    // check if the message is for this motor
    if (!check_msg_id(msg))
        return 0;

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
            m_motor_exception = *((uint64_t *)&msg.buf[0]);
            break;
        }
        case ErrorRequestType::ENCODER: {
            m_encoder_exception = *((uint32_t *)&msg.buf[0]);
            break;
        }
        case ErrorRequestType::CONTROL: {
            m_control_exception = *((uint32_t *)&msg.buf[0]);
            break;
        }
        case ErrorRequestType::SYSTEM: {
            m_system_exception = *((uint32_t *)&msg.buf[0]);
            break;
        }
        default: {
            SystemLog.error(Subsystem::MOTORS, "Unknown error type: %d\n", static_cast<int>(m_error_request_type));
            break;
        }
        }
        break;
    }
    case CMD_MIT_CONTROL: {
        SystemLog.error(Subsystem::MOTORS, "MIT control command not implemented\n");
        break;
    }
    case CMD_GET_ENCODER_ESTIMATES: {
        m_position_estimate = *((float *)&msg.buf[0]);
        m_velocity_estimate = *((float *)&msg.buf[4]);

        // update the unified state struct. The encoder is on the motor side of the gearbox and gives revolutions,
        // we want radians at the output
        m_position_rad = m_position_estimate * 2 * PI / m_gear_ratio;
        m_state.speed = m_velocity_estimate * 2 * PI / m_gear_ratio;
        // the unified position is a 16 bit count over one output revolution, like the other drivers' encoder counts
        float output_rev = m_position_estimate / m_gear_ratio;
        m_state.position = (uint16_t)((output_rev - floorf(output_rev)) * 65536.0f);
        break;
    }
    case CMD_GET_ENCODER_COUNT: {
        m_encoder_shadow_count = *((int32_t *)&msg.buf[0]);
        m_encoder_revolution_count = *((int32_t *)&msg.buf[4]);
        break;
    }
    case CMD_GET_IQ: {
        m_iq_setpoint = *((float *)&msg.buf[0]);
        m_iq_measured = *((float *)&msg.buf[4]);
        break;
    }
    case CMD_GET_BUS_VOLTAGE_CURRENT: {
        m_bus_voltage = *((float *)&msg.buf[0]);
        m_bus_current = *((float *)&msg.buf[4]);
        break;
    }
    case CMD_GET_TORQUES: {
        m_torque_setpoint = *((float *)&msg.buf[0]);
        m_torque_measured = *((float *)&msg.buf[4]);

        // update the unified state struct: motor side torque to output torque, normalized to [-1, 1]
        m_state.torque = m_torque_measured * m_gear_ratio / m_max_torque;
        break;
    }
    case CMD_GET_POWERS: {
        m_electrical_power = *((float *)&msg.buf[0]);
        m_mechanical_power = *((float *)&msg.buf[4]);
        break;
    }
    default: {
        SystemLog.error(Subsystem::MOTORS, "Unknown command byte: %d\n", cmd_byte);
        break;
    }
    }

    return 1;
}

int SDC104::write(CAN_message_t &msg) const {
    // queued one-shot commands go out first, each exactly once
    if (m_one_shot_count > 0) {
        memcpy(&msg, &m_one_shots[m_one_shot_head], sizeof(CAN_message_t));
        m_one_shot_head = (m_one_shot_head + 1) % ONE_SHOT_QUEUE_SIZE;
        m_one_shot_count--;
        return 0;
    }

    // otherwise alternate: command, encoder estimates, command, torques, ...
    switch (m_write_count++ % 4) {
    case 1:
        create_cmd_get_encoder_estimates(msg);
        break;
    case 3:
        create_cmd_get_torques(msg);
        break;
    default:
        if (m_have_command)
            memcpy(&msg, &m_output, sizeof(CAN_message_t));
        // nothing commanded yet: a zero torque input does nothing until it's in closed loop torque control
        else
            create_cmd_set_input_torque(msg, 0.0f);
        break;
    }

    return 0;
}

void SDC104::set_output(const CAN_message_t &msg, bool one_shot) {
    if (!one_shot) {
        memcpy(&m_output, &msg, sizeof(CAN_message_t));
        m_have_command = true;
        return;
    }

    if (m_one_shot_count >= ONE_SHOT_QUEUE_SIZE) {
        SystemLog.error(Subsystem::MOTORS, "SDC104 %u: one-shot queue full, dropping command 0x%02lX\n", m_physical_id, msg.id & 0x1F);
        return;
    }
    uint8_t tail = (m_one_shot_head + m_one_shot_count) % ONE_SHOT_QUEUE_SIZE;
    memcpy(&m_one_shots[tail], &msg, sizeof(CAN_message_t));
    m_one_shot_count++;
}

bool SDC104::prepare_for(ControlMode control_mode) {
    // let queued mode/axis commands go out before deciding anything else
    if (m_one_shot_count > 0)
        return false;

    if (!m_control_mode_known || m_control_mode != control_mode) {
        set_controller_mode(control_mode, InputMode::DIRECT);
        m_control_mode = control_mode;
        m_control_mode_known = true;
        return false;
    }
    // the motor won't accept torque or speed commands in any other state. The heartbeat that confirms closed loop
    // takes up to ~100 ms, so don't re-request it every loop in the meantime
    if (m_heartbeat.axis_state != AxisState::CLOSED_LOOP_CONTROL) {
        if (millis() - m_closed_loop_requested_ms > CLOSED_LOOP_RETRY_MS) {
            set_axis_state(AxisState::CLOSED_LOOP_CONTROL);
            m_closed_loop_requested_ms = millis();
        }
        return false;
    }
    return true;
}

void SDC104::zero_motor() { execute_motor_torque_command(0.0f); }

void SDC104::execute_motor_torque_command(float torque) {
    // clamp the torque value
    if (torque < -1.0f)
        torque = -1.0f;
    if (torque > 1.0f)
        torque = 1.0f;

    if (!prepare_for(ControlMode::TORQUE))
        return;

    // map the -1f to 1f torque value to output torque, then to the motor side torque the SDC104 takes
    float motor_torque = torque * m_max_torque / m_gear_ratio;

    CAN_message_t msg;
    create_cmd_set_input_torque(msg, motor_torque);
    set_output(msg, false);
}

void SDC104::write_motor_speed(float speed) {
    if (speed < -1.0f)
        speed = -1.0f;
    if (speed > 1.0f)
        speed = 1.0f;

    if (!prepare_for(ControlMode::VELOCITY))
        return;

    // map the -1f to 1f speed value to output RPM, then to the motor side rev/s the SDC104 takes
    float motor_rev_per_s = speed * m_max_speed / 60.0f * m_gear_ratio;

    CAN_message_t msg;
    create_cmd_set_input_velocity(msg, motor_rev_per_s, 0);
    set_output(msg, false);
}

void SDC104::print_state() const {
    Motor::print_state();
    Serial.printf("Axis state: %d\n", static_cast<int>(m_heartbeat.axis_state));
}

void SDC104::get_error(uint8_t error_type) {
    CAN_message_t msg;
    create_cmd_get_error(msg, error_type);

    set_output(msg);
}

void SDC104::set_axis_state(AxisState axis_state) {
    CAN_message_t msg;
    create_cmd_set_axis_state(msg, axis_state);

    set_output(msg);
}

void SDC104::get_encoder_count() {
    CAN_message_t msg;
    create_cmd_get_encoder_count(msg);

    set_output(msg);
}

void SDC104::set_limits(float velocity_limit, float current_limit) {
    CAN_message_t msg;
    create_cmd_set_limits(msg, velocity_limit, current_limit);

    set_output(msg);
}

void SDC104::set_controller_mode(ControlMode control_mode, InputMode input_mode) {
    CAN_message_t msg;
    create_cmd_set_controller_mode(msg, static_cast<uint32_t>(control_mode), static_cast<uint32_t>(input_mode));

    set_output(msg);
}

void SDC104::get_bus_voltage_current() {
    CAN_message_t msg;
    create_cmd_get_bus_voltage_current(msg);

    set_output(msg);
}

void SDC104::clear_errors() {
    CAN_message_t msg;
    create_cmd_clear_errors(msg);

    set_output(msg);
}

void SDC104::set_linear_count(int32_t linear_count) {
    CAN_message_t msg;
    create_cmd_set_linear_count(msg, linear_count);

    set_output(msg);
}

void SDC104::set_position_gains(float pos_gain) {
    CAN_message_t msg;
    create_cmd_set_position_gains(msg, pos_gain);

    set_output(msg);
}

void SDC104::set_velocity_gains(float vel_gain, float vel_integrator_gain) {
    CAN_message_t msg;
    create_cmd_set_velocity_gains(msg, vel_gain, vel_integrator_gain);

    set_output(msg);
}

void SDC104::save_configuration() {
    CAN_message_t msg;
    create_cmd_save_configuration(msg);

    set_output(msg);
}

void SDC104::reboot() {
    CAN_message_t msg;
    create_cmd_reboot(msg);

    set_output(msg);
}

bool SDC104::check_msg_id(const CAN_message_t &msg) const {
    // last 6 bits of the ID are the motor ID
    uint8_t id = (msg.id >> 5) & 0x3F;

    // check if the ID is correct
    if (id != m_base_id + m_physical_id) {
        return false;
    }

    // check if the bus is correct
    if ((uint32_t)(msg.bus - 1) != m_bus_id) {
        return false;
    }

    return true;
}

void SDC104::create_cmd_estop(CAN_message_t &msg) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_ESTOP;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_get_error(CAN_message_t &msg, uint8_t error_type) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_GET_ERROR;
    memset(msg.buf, 0, 8);

    msg.buf[0] = error_type;

    // set the error request type
    m_error_request_type = static_cast<ErrorRequestType>(error_type);
}

void SDC104::create_cmd_set_axis_node_id(CAN_message_t &msg, uint32_t node_id) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_AXIS_NODE_ID;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&node_id + 0);
    msg.buf[1] = *((uint8_t *)&node_id + 1);
    msg.buf[2] = *((uint8_t *)&node_id + 2);
    msg.buf[3] = *((uint8_t *)&node_id + 3);
}

void SDC104::create_cmd_set_axis_state(CAN_message_t &msg, AxisState axis_state) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_AXIS_STATE;
    memset(msg.buf, 0, 8);

    msg.buf[0] = static_cast<uint8_t>(axis_state);
}

void SDC104::create_cmd_get_encoder_estimates(CAN_message_t &msg) const {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_GET_ENCODER_ESTIMATES;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_get_encoder_count(CAN_message_t &msg) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_GET_ENCODER_COUNT;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_set_controller_mode(CAN_message_t &msg, uint32_t control_mode, uint32_t input_mode) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_CONTROLLER_MODE;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&control_mode + 0);
    msg.buf[1] = *((uint8_t *)&control_mode + 1);
    msg.buf[2] = *((uint8_t *)&control_mode + 2);
    msg.buf[3] = *((uint8_t *)&control_mode + 3);

    msg.buf[4] = *((uint8_t *)&input_mode + 0);
    msg.buf[5] = *((uint8_t *)&input_mode + 1);
    msg.buf[6] = *((uint8_t *)&input_mode + 2);
    msg.buf[7] = *((uint8_t *)&input_mode + 3);
}

void SDC104::create_cmd_set_input_position(CAN_message_t &msg, float input_position, int16_t velocity_ff, int16_t torque_ff) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_INPUT_POSITION;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&input_position + 0);
    msg.buf[1] = *((uint8_t *)&input_position + 1);
    msg.buf[2] = *((uint8_t *)&input_position + 2);
    msg.buf[3] = *((uint8_t *)&input_position + 3);

    msg.buf[4] = *((uint8_t *)&velocity_ff + 0);
    msg.buf[5] = *((uint8_t *)&velocity_ff + 1);

    msg.buf[6] = *((uint8_t *)&torque_ff + 0);
    msg.buf[7] = *((uint8_t *)&torque_ff + 1);
}

void SDC104::create_cmd_set_input_velocity(CAN_message_t &msg, float input_velocity, int16_t torque_ff) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_INPUT_VELOCITY;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&input_velocity + 0);
    msg.buf[1] = *((uint8_t *)&input_velocity + 1);
    msg.buf[2] = *((uint8_t *)&input_velocity + 2);
    msg.buf[3] = *((uint8_t *)&input_velocity + 3);

    msg.buf[4] = *((uint8_t *)&torque_ff + 0);
    msg.buf[5] = *((uint8_t *)&torque_ff + 1);
}

void SDC104::create_cmd_set_input_torque(CAN_message_t &msg, float input_torque) const {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_INPUT_TORQUE;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&input_torque + 0);
    msg.buf[1] = *((uint8_t *)&input_torque + 1);
    msg.buf[2] = *((uint8_t *)&input_torque + 2);
    msg.buf[3] = *((uint8_t *)&input_torque + 3);
}

void SDC104::create_cmd_set_limits(CAN_message_t &msg, float velocity_limit, float current_limit) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_LIMITS;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&velocity_limit + 0);
    msg.buf[1] = *((uint8_t *)&velocity_limit + 1);
    msg.buf[2] = *((uint8_t *)&velocity_limit + 2);
    msg.buf[3] = *((uint8_t *)&velocity_limit + 3);

    msg.buf[4] = *((uint8_t *)&current_limit + 0);
    msg.buf[5] = *((uint8_t *)&current_limit + 1);
    msg.buf[6] = *((uint8_t *)&current_limit + 2);
    msg.buf[7] = *((uint8_t *)&current_limit + 3);
}

void SDC104::create_cmd_start_anticogging(CAN_message_t &msg) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_START_ANTICOGGING;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_set_traj_velocity_limit(CAN_message_t &msg, float traj_velocity_limit) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_TRAJ_VELOCITY_LIMIT;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&traj_velocity_limit + 0);
    msg.buf[1] = *((uint8_t *)&traj_velocity_limit + 1);
    msg.buf[2] = *((uint8_t *)&traj_velocity_limit + 2);
    msg.buf[3] = *((uint8_t *)&traj_velocity_limit + 3);
}

void SDC104::create_cmd_set_traj_acceleration_limit(CAN_message_t &msg, float traj_acceleration_limit, float traj_deceleration_limit) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_TRAJ_ACCELERATION_LIMIT;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&traj_acceleration_limit + 0);
    msg.buf[1] = *((uint8_t *)&traj_acceleration_limit + 1);
    msg.buf[2] = *((uint8_t *)&traj_acceleration_limit + 2);
    msg.buf[3] = *((uint8_t *)&traj_acceleration_limit + 3);

    msg.buf[4] = *((uint8_t *)&traj_deceleration_limit + 0);
    msg.buf[5] = *((uint8_t *)&traj_deceleration_limit + 1);
    msg.buf[6] = *((uint8_t *)&traj_deceleration_limit + 2);
    msg.buf[7] = *((uint8_t *)&traj_deceleration_limit + 3);
}

void SDC104::create_cmd_set_traj_inertia(CAN_message_t &msg, float traj_inertia) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_TRAJ_INERTIA;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&traj_inertia + 0);
    msg.buf[1] = *((uint8_t *)&traj_inertia + 1);
    msg.buf[2] = *((uint8_t *)&traj_inertia + 2);
    msg.buf[3] = *((uint8_t *)&traj_inertia + 3);
}

void SDC104::create_cmd_get_iq(CAN_message_t &msg) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_GET_IQ;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_reboot(CAN_message_t &msg) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_REBOOT;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_get_bus_voltage_current(CAN_message_t &msg) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_GET_BUS_VOLTAGE_CURRENT;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_clear_errors(CAN_message_t &msg) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_CLEAR_ERRORS;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_set_linear_count(CAN_message_t &msg, int32_t linear_count) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_LINEAR_COUNT;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&linear_count + 0);
    msg.buf[1] = *((uint8_t *)&linear_count + 1);
    msg.buf[2] = *((uint8_t *)&linear_count + 2);
    msg.buf[3] = *((uint8_t *)&linear_count + 3);
}

void SDC104::create_cmd_set_position_gains(CAN_message_t &msg, float pos_gain) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_POSITION_GAINS;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&pos_gain + 0);
    msg.buf[1] = *((uint8_t *)&pos_gain + 1);
    msg.buf[2] = *((uint8_t *)&pos_gain + 2);
    msg.buf[3] = *((uint8_t *)&pos_gain + 3);
}

void SDC104::create_cmd_set_velocity_gains(CAN_message_t &msg, float vel_gain, float vel_integrator_gain) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SET_VELOCITY_GAINS;
    memset(msg.buf, 0, 8);

    msg.buf[0] = *((uint8_t *)&vel_gain + 0);
    msg.buf[1] = *((uint8_t *)&vel_gain + 1);
    msg.buf[2] = *((uint8_t *)&vel_gain + 2);
    msg.buf[3] = *((uint8_t *)&vel_gain + 3);

    msg.buf[4] = *((uint8_t *)&vel_integrator_gain + 0);
    msg.buf[5] = *((uint8_t *)&vel_integrator_gain + 1);
    msg.buf[6] = *((uint8_t *)&vel_integrator_gain + 2);
    msg.buf[7] = *((uint8_t *)&vel_integrator_gain + 3);
}

void SDC104::create_cmd_get_torques(CAN_message_t &msg) const {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_GET_TORQUES;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_get_powers(CAN_message_t &msg) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_GET_POWERS;
    memset(msg.buf, 0, 8);
}

void SDC104::create_cmd_save_configuration(CAN_message_t &msg) {
    msg.id = ((m_base_id + m_physical_id) << 5) | CMD_SAVE_CONFIGURATION;
    memset(msg.buf, 0, 8);
}
