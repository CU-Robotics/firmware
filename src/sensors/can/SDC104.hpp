#pragma once

#include "motor.hpp"

/// @brief SDC104 motor controller class, driving a GIM6010-8
/// @note The SDC104 is running firmware version 0.5.13(1), ODrive based. Its CAN units are on the motor side of the
/// 8:1 gearbox: encoder revolutions are motor revolutions and torque is motor torque. This driver converts so that
/// everything it takes and reports is at the output, like the other motors
class SDC104 : public Motor {
public:
    /// @brief Deleted default constructor, must explicitly construct this object. Incomplete objects are not allowed
    SDC104() = delete;

    /// @brief Main constructor. Defines the controller type, global ID, id, can bus, and motor type
    /// @param motor_config The configuration for this motor, including its controller type, physical bus and id, motor type, and motor name.
    SDC104(Cfg::Motor motor_config)
        : Motor(motor_config) {
    }

    /// @brief Destructor, does nothing
    ~SDC104() override { }

public:
    /// @brief The current driver state/mode
    enum class AxisState : uint8_t {
        UNDEFINED = 0,
        IDLE = 1,
        FULL_CALIBRATION = 3,
        MOTOR_CALIBRATION = 4,
        ENCODER_CALIBRATION = 7,
        CLOSED_LOOP_CONTROL = 8
    };

    /// @brief The current control mode
    enum class ControlMode : uint32_t {
        VOLTAGE = 0,
        TORQUE = 1,
        VELOCITY = 2,
        POSITION = 3
    };

    /// @brief The current input mode
    enum class InputMode : uint32_t {
        IDLE = 0,
        DIRECT = 1,
        SPEED_RAMP = 2,
        POSITION_FILTERING = 3,
        TRAPEZOIDAL_CURVES = 5,
        TORQUE_RAMP = 6,
        MIT_CONTROL = 9
    };

public:
    /// @brief Initialize the motor by putting it in idle
    /// @note The SDC104 keeps executing its last command forever, so a Teensy reset in the middle of a move would
    /// otherwise leave it running. The first torque/speed command puts it back in closed loop
    void init() override;

    /// @brief Common read command. Fills given message if successful
    /// @param msg The message buffer to fill data into
    /// @return 0 on failure, 1 on success
    int read(CAN_message_t& msg) override;

    /// @brief Common write command. This fills the given message if successful. This is used to compile data to be sent over the CAN line
    /// @param msg The message buffer to fill data into
    /// @return The index in the buffer where the motor data was written. The SDC104 driver will always write 8 bytes so this will always return 0
    /// @note Does not issue a CAN command over the bus
    /// @note The SDC104 only sends heartbeats on its own, so status has to be requested. A new one-shot command
    /// (mode, axis state, ...) is sent first; otherwise calls alternate between the current torque/speed command,
    /// which the SDC104 holds until the next one, and a status request (encoder estimates or torques)
    int write(CAN_message_t& msg) const override;

    /// @brief Zero the motor. This is a safety function to ensure the motor is not actively driving
    /// @note Does not issue a CAN command over the bus
    void zero_motor() override;

    /// @brief Write motor torque given a normalized value
    /// @param torque A value between [-1, 1] of the max output torque. Switches the motor to torque control and
    /// closed loop first if needed, in which case the torque is sent on a later call
    void execute_motor_torque_command(float torque) override;

    /// @brief Write motor speed given a normalized value
    /// @param speed A value between [-1, 1] of the max output speed. Switches the motor to velocity control and
    /// closed loop first if needed, in which case the speed is sent on a later call
    void write_motor_speed(float speed);

    /// @brief Get the last reported output shaft position
    /// @return Multi-turn position in radians at the output
    inline float get_position_rad() const { return m_position_rad; }

    /// @brief Print the current state of the motor
    void print_state() const override;

public:
    /// @brief Get the motor error state
    /// @param error_type 0: motor abnormality, 1: encoder exception, 3: control exception, 4: system exception
    void get_error(uint8_t error_type);

    /// @brief Set the axis state
    /// @param axis_state The new axis state
    void set_axis_state(AxisState axis_state);

    /// @brief Get the encoder count estimates
    /// @note Encoder actual position/velocity estimate is grabbed through periodic heartbeat messages
    void get_encoder_count();

    /// @brief Set the velocity and current limits
    /// @param velocity_limit The new velocity limit in rev/s
    /// @param current_limit The new current limit in A
    void set_limits(float velocity_limit, float current_limit);

    /// @brief Set the controller mode
    /// @param control_mode control type
    /// @param input_mode input type
    void set_controller_mode(ControlMode control_mode, InputMode input_mode);

    /// @brief Get the voltage and current on the bus
    void get_bus_voltage_current();

    /// @brief Clear any errors on the motor
    void clear_errors();

    /// @brief Set the linear count of the encoder
    /// @param linear_count The new absolute encoder position in counts
    void set_linear_count(int32_t linear_count);

    /// @brief Set the position gains
    /// @param pos_gain The new position gain in (rev/s)/rev
    void set_position_gains(float pos_gain);

    /// @brief Set the velocity gains
    /// @param vel_gain The new velocity gain in Nm/(rev/s)
    /// @param vel_integrator_gain The new velocity integrator gain in Nm/rev
    void set_velocity_gains(float vel_gain, float vel_integrator_gain);

    /// @brief Save the current configuration to the motor's flash memory
    /// @note This does not seem to work
    void save_configuration();

    /// @brief Reboot the motor
    void reboot();

private:
    /// @brief Put a command into the output frame
    /// @param msg The command to send
    /// @param one_shot True for commands that only need to be sent once (mode, axis state, ...), false for the
    /// torque/speed commands that are resent every loop
    void set_output(const CAN_message_t& msg, bool one_shot = true);

    /// @brief Get the motor ready for torque or speed commands: right control mode, closed loop
    /// @param control_mode The control mode the next command needs
    /// @return True if it's ready, false if a mode or axis state command was staged instead
    bool prepare_for(ControlMode control_mode);

    /// @brief Check if the message is for this motor
    /// @param msg The message to check
    /// @return True if the message is for this motor, false otherwise
    bool check_msg_id(const CAN_message_t& msg) const override;

    /// @brief Create an emergency stop command
    /// @param msg The msg to fill in
    void create_cmd_estop(CAN_message_t& msg);

    /// @brief Create a get error command
    /// @param msg The msg to fill in
    /// @param error_type 0: motor abnormality, 1: encoder exception, 3: control exception, 4: system exception
    void create_cmd_get_error(CAN_message_t& msg, uint8_t error_type);

    // unimplemented
    // void create_cmd_rxsdo(CAN_message_t& msg, uint8_t opcode, uint16_t endpoint_id, uint8_t data[4]);

    // unimplemented
    // void create_cmd_txsdo(CAN_message_t& msg, uint8_t opcode, uint16_t endpoint_id, uint8_t data[4]);

    /// @brief Create a set axis node ID command
    /// @param msg The msg to fill in
    /// @param node_id The new unique CAN node ID
    void create_cmd_set_axis_node_id(CAN_message_t& msg, uint32_t node_id);

    /// @brief Create a set axis state command
    /// @param msg The msg to fill in
    /// @param axis_state The new axis state
    /// 0: undefined, 1: idle, 3: Full calibration, 4: motor calibration, 7: encoder calibration, 8: closed loop control
    void create_cmd_set_axis_state(CAN_message_t& msg, AxisState axis_state);

    // unimplemented
    // void create_cmd_mit_control(CAN_message_t& msg);

    /// @brief Create a get encoder estimates command
    /// @param msg The msg to fill in
    void create_cmd_get_encoder_estimates(CAN_message_t& msg) const;

    /// @brief Create a get encoder count command
    /// @param msg The msg to fill in
    void create_cmd_get_encoder_count(CAN_message_t& msg);

    /// @brief Create a set controller mode command
    /// @param msg The msg to fill in
    /// @param control_mode 0: voltage control, 1: torque control, 2: velocity control, 3: position control
    /// @param input_mode 0: idle, 1: direct, 2: speed ramp, 3: position filtering, 5: trapezoidal curves, 6: torque ramp, 9: MIT control
    void create_cmd_set_controller_mode(CAN_message_t& msg, uint32_t control_mode, uint32_t input_mode);

    /// @brief Create a set input position command
    /// @param msg The msg to fill in
    /// @param input_position The new input position in rev
    /// @param velocity_ff The velocity feedforward in 0.001rev/s
    /// @param torque_ff The torque feedforward in 0.001Nm
    void create_cmd_set_input_position(CAN_message_t& msg, float input_position, int16_t velocity_ff, int16_t torque_ff);

    /// @brief Create a set input velocity command
    /// @param msg The msg to fill in
    /// @param input_velocity The new input velocity in rev/s
    /// @param torque_ff The torque feedforward in Nm
    void create_cmd_set_input_velocity(CAN_message_t& msg, float input_velocity, int16_t torque_ff);

    /// @brief Create a set input torque command
    /// @param msg The msg to fill in
    /// @param input_torque The new input torque in Nm
    void create_cmd_set_input_torque(CAN_message_t& msg, float input_torque);

    /// @brief Create a set limits command
    /// @param msg The msg to fill in
    /// @param velocity_limit The new velocity limit in rev/s
    /// @param current_limit The new current limit in A
    void create_cmd_set_limits(CAN_message_t& msg, float velocity_limit, float current_limit);

    /// @brief Create a start anticogging command
    /// @param msg The msg to fill in
    void create_cmd_start_anticogging(CAN_message_t& msg);

    /// @brief Create a set trajectory velocity limit command
    /// @param msg The msg to fill in
    /// @param traj_velocity_limit The new trajectory velocity limit in rev/s
    void create_cmd_set_traj_velocity_limit(CAN_message_t& msg, float traj_velocity_limit);

    /// @brief Create a set trajectory acceleration limit command
    /// @param msg The msg to fill in
    /// @param traj_acceleration_limit The new trajectory acceleration limit in rev/s^2
    /// @param traj_deceleration_limit The new trajectory deceleration limit in rev/s^2
    void create_cmd_set_traj_acceleration_limit(CAN_message_t& msg, float traj_acceleration_limit, float traj_deceleration_limit);

    /// @brief Create a set trajectory inertia command
    /// @param msg The msg to fill in
    /// @param traj_inertia The new trajectory inertia in Nm/(rev/s^2)
    void create_cmd_set_traj_inertia(CAN_message_t& msg, float traj_inertia);

    /// @brief Create a get IQ command
    /// @param msg The msg to fill in
    void create_cmd_get_iq(CAN_message_t& msg);

    /// @brief Create a reboot command
    /// @param msg The msg to fill in
    void create_cmd_reboot(CAN_message_t& msg);

    /// @brief Create a get bus voltage and current command
    /// @param msg The msg to fill in
    void create_cmd_get_bus_voltage_current(CAN_message_t& msg);

    /// @brief Create a clear errors command
    /// @param msg The msg to fill in
    void create_cmd_clear_errors(CAN_message_t& msg);

    /// @brief Create a set linear count command
    /// @param msg The msg to fill in
    /// @param linear_count The new absolute encoder position in counts
    void create_cmd_set_linear_count(CAN_message_t& msg, int32_t linear_count);

    /// @brief Create a set position gains command
    /// @param msg The msg to fill in
    /// @param pos_gain The new position gain in (rev/s)/rev
    void create_cmd_set_position_gains(CAN_message_t& msg, float pos_gain);

    /// @brief Create a set velocity gains command
    /// @param msg The msg to fill in
    /// @param vel_gain The new velocity gain in Nm/(rev/s)
    /// @param vel_integrator_gain The new velocity integrator gain in Nm/rev
    void create_cmd_set_velocity_gains(CAN_message_t& msg, float vel_gain, float vel_integrator_gain);

    /// @brief Create a get torques command
    /// @param msg The msg to fill in
    void create_cmd_get_torques(CAN_message_t& msg) const;

    /// @brief Create a get powers command
    /// @param msg The msg to fill in
    void create_cmd_get_powers(CAN_message_t& msg);

    // unimplemented
    // void create_cmd_disable_can(CAN_message_t& msg);

    /// @brief Create a save configuration command
    /// @param msg The msg to fill in
    void create_cmd_save_configuration(CAN_message_t& msg);

private:
    /// @brief The motor's max torque in Nm at the output
    float m_max_torque = 11.0f;

    /// @brief The motor's gear ratio, 8:1. The SDC104's units are on the motor side of it
    float m_gear_ratio = 8.0f;

    /// @brief The motor's maximum speed in RPM at the output
    float m_max_speed = 420.0f;

    /// @brief The control mode last sent to the motor, so it's only sent when it changes
    ControlMode m_control_mode = ControlMode::VOLTAGE;
    /// @brief Whether m_control_mode has been sent at all
    bool m_control_mode_known = false;

    /// @brief Whether m_output is a one-shot command (mode, axis state, ...) rather than a resent torque/speed command
    bool m_output_one_shot = false;
    /// @brief Whether m_output has been sent since it was last set
    mutable bool m_output_sent = false;
    /// @brief Counts write() calls, to alternate between the command and status requests
    mutable uint32_t m_write_count = 0;

    /// @brief Last reported multi-turn output shaft position in radians
    float m_position_rad = 0.0f;

    // Motor specific state
    /// @brief Motor heartbeat data
    struct {
        /// @brief driver exception code
        uint32_t axis_error;
        /// @brief driver state
        AxisState axis_state;
        /// @brief motor error status
        uint8_t motor_exception : 1;
        /// @brief encoder error status
        uint8_t encoder_exception : 1;
        /// @brief controller error status
        uint8_t control_exception : 1;
        /// @brief system error status
        uint8_t system_exception : 1;
        // pad
        uint8_t : 3;
        /// @brief is the trajectory done
        uint8_t trajectory_done : 1;
        // reserved
        uint8_t : 8;
        /// @brief consecuitive "life" sequence number
        uint8_t life_seq;
    } m_heartbeat = {};

    /// @brief What error type is currently being requested. 0: motor abnormality, 1: encoder exception, 3: control exception, 4: system exception
    enum class ErrorRequestType {
        MOTOR = 0,
        ENCODER = 1,
        CONTROL = 3,
        SYSTEM = 4
    };
    /// @brief The current error request type
    ErrorRequestType m_error_request_type = ErrorRequestType::MOTOR;

    /// @brief The motor error state
    uint64_t m_motor_exception = 0;
    /// @brief The encoder error state
    uint32_t m_encoder_exception = 0;
    /// @brief The control error state
    uint32_t m_control_exception = 0;
    /// @brief The system error state
    uint32_t m_system_exception = 0;

    /// @brief Current motor position estimate in rev
    /// @note This is the motor side of the gearbox, not the output
    float m_position_estimate = 0;
    /// @brief Current motor velocity estimate in rev/s
    /// @note This is the motor side of the gearbox, not the output
    float m_velocity_estimate = 0;

    /// @brief Encoder multi-turn count
    int32_t m_encoder_shadow_count = 0;
    /// @brief Encoder single turn count
    int32_t m_encoder_revolution_count = 0;

    /// @brief Target Q-axis current in A
    float m_iq_setpoint = 0;
    /// @brief Actual Q-axis current in A
    float m_iq_measured = 0;

    /// @brief Current bus voltage in V
    float m_bus_voltage = 0;
    /// @brief Current bus current in A
    float m_bus_current = 0;

    /// @brief The motor torque setpoint in Nm, motor side
    float m_torque_setpoint = 0;
    /// @brief The motor torque measured in Nm, motor side
    float m_torque_measured = 0;

    /// @brief The electrical power in W
    float m_electrical_power = 0;
    /// @brief The mechanical power in W
    float m_mechanical_power = 0;

    // Command command byte definitions

    /// @brief Heartbeat cmd byte
    static constexpr uint8_t CMD_HEARTBEAT = 0x01;
    /// @brief Emergency stop cmd byte
    static constexpr uint8_t CMD_ESTOP = 0x02;
    /// @brief Get error cmd byte
    static constexpr uint8_t CMD_GET_ERROR = 0x03;
    /// @brief Receive SDO cmd byte
    static constexpr uint8_t CMD_RXSDO = 0x04;
    /// @brief Transmit SDO cmd byte
    static constexpr uint8_t CMD_TXSDO = 0x05;
    /// @brief Set axis node ID cmd byte
    static constexpr uint8_t CMD_SET_AXIS_NODE_ID = 0x06;
    /// @brief Set axis state cmd byte
    static constexpr uint8_t CMD_SET_AXIS_STATE = 0x07;
    /// @brief MIT control cmd byte
    static constexpr uint8_t CMD_MIT_CONTROL = 0x08;
    /// @brief Get encoder estimates cmd byte
    static constexpr uint8_t CMD_GET_ENCODER_ESTIMATES = 0x09;
    /// @brief Get encoder count cmd byte
    static constexpr uint8_t CMD_GET_ENCODER_COUNT = 0x0A;
    /// @brief Set controller mode cmd byte
    static constexpr uint8_t CMD_SET_CONTROLLER_MODE = 0x0B;
    /// @brief Set input position cmd byte
    static constexpr uint8_t CMD_SET_INPUT_POSITION = 0x0C;
    /// @brief Set input velocity cmd byte
    static constexpr uint8_t CMD_SET_INPUT_VELOCITY = 0x0D;
    /// @brief Set input torque cmd byte
    static constexpr uint8_t CMD_SET_INPUT_TORQUE = 0x0E;
    /// @brief Set limits cmd byte
    static constexpr uint8_t CMD_SET_LIMITS = 0x0F;
    /// @brief Start anticogging cmd byte
    static constexpr uint8_t CMD_START_ANTICOGGING = 0x10;
    /// @brief Set trajectory velocity limit cmd byte
    static constexpr uint8_t CMD_SET_TRAJ_VELOCITY_LIMIT = 0x11;
    /// @brief Set trajectory acceleration limit cmd byte
    static constexpr uint8_t CMD_SET_TRAJ_ACCELERATION_LIMIT = 0x12;
    /// @brief Set trajectory inertia cmd byte
    static constexpr uint8_t CMD_SET_TRAJ_INERTIA = 0x13;
    /// @brief Get IQ cmd byte
    static constexpr uint8_t CMD_GET_IQ = 0x14;
    /// @brief Reboot cmd byte
    static constexpr uint8_t CMD_REBOOT = 0x16;
    /// @brief Get bus voltage and current cmd byte
    static constexpr uint8_t CMD_GET_BUS_VOLTAGE_CURRENT = 0x17;
    /// @brief Clear errors cmd byte
    static constexpr uint8_t CMD_CLEAR_ERRORS = 0x18;
    /// @brief Set linear count cmd byte
    static constexpr uint8_t CMD_SET_LINEAR_COUNT = 0x19;
    /// @brief Set position gains cmd byte
    static constexpr uint8_t CMD_SET_POSITION_GAINS = 0x1A;
    /// @brief Set velocity gains cmd byte
    static constexpr uint8_t CMD_SET_VELOCITY_GAINS = 0x1B;
    /// @brief Get torques cmd byte
    static constexpr uint8_t CMD_GET_TORQUES = 0x1C;
    /// @brief Get powers cmd byte
    static constexpr uint8_t CMD_GET_POWERS = 0x1D;
    /// @brief Disable CAN cmd byte
    static constexpr uint8_t CMD_DISABLE_CAN = 0x1E;
    /// @brief Save configuration cmd byte
    static constexpr uint8_t CMD_SAVE_CONFIGURATION = 0x1F;

};