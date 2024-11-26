#ifndef MG8016EI6_DRIVER_HPP
#define MG8016EI6_DRIVER_HPP

#include "motor.hpp"

// MG8016E-i6 motor data sheet
// http://en.lkmotor.cn/upload/20230321150047f.pdf

// MG8016E-i6 CAN protocol data sheet
// http://en.lkmotor.cn/upload/20230706100134f.pdf

/// @brief MG8016E-i6v3 controller driver. This manages generating CAN output messages and processing incomming CAN messages into a state array.
/// @note It's construction is heavily managed since copying this object could alter the actions of CAN (and by extension the robot). This object exists only to be managed by CANManager.
class MG8016EI6 : public Motor {
public:
    /// @brief Deleted default constructor, must explicitly construct this object. Incomplete objects are not allowed
    MG8016EI6() = delete;

    /// @brief Main constructor. Defines the controller type, global ID, id, and can bus
    /// @param gid The global ID, not the per-bus motor ID
    /// @param id The per-bus motor ID. This is 1-indexed
    /// @param bus_id The CAN bus index/ID
    MG8016EI6(uint32_t gid, uint32_t id, uint8_t bus_id)
        : Motor(MG8016_CONTROLLER, gid, id, bus_id) {
    }

    /// @brief Deleted copy constructor, you must not copy this object
    /// @param copy copy
    MG8016EI6(const MG8016EI6& copy) = delete;
    /// @brief Deleted copy assignment operator, you must not move/copy this object
    /// @param copy copy
    /// @return MG8016E_I6V3&
    MG8016EI6& operator=(const MG8016EI6& copy) = delete;

    /// @brief Destructor, does nothing
    ~MG8016EI6() override { }

public:
    /// @brief Common read command. Fills given message if successful
    /// @param msg The message buffer to fill data into
    /// @return 0 on failure, 1 on success
    int read(CAN_message_t& msg) override;

    /// @brief Common write command. This fills the given message if successful. This is used to compile data to be sent over the CAN line
    /// @param msg The message buffer to fill data into
    /// @return 0 on failure, 1 on success
    /// @note Does not issue a CAN command over the bus
    int write(CAN_message_t& msg) override;

    /// @brief Write motor torque given a normalized value
    /// @param torque A value between [-1, 1] representing the torque range of [-33A, 33A]
    void write_motor_torque(float torque) override;

    /// @brief Write motor speed
    /// @param speed The speed value in radians per second
    void write_motor_speed(float speed);

    /// @brief Write motor angle
    /// @param angle The angle value in radians
    /// @param speed_limit The speed limit in radians per second, 0 is no limit, must be positive
    void write_motor_angle(float angle, float speed_limit = 0);

    /// @brief Print the current state of the motor
    void print_state() override;

public:
    // TODO: figure out the best way for these to be called since they need to issue a command pretty much immidiately

    /// @brief Turn off the motor and clear it's state
    void write_motor_off();

    /// @brief Turn on the motor
    void write_motor_on();

    /// @brief Stop the motor but dont clear it's state
    void write_motor_stop();

private:
    /// @brief Turn off the motor and clear it's state
    /// @param buf Output buffer to write the command to
    void create_cmd_motor_off(uint8_t buf[8]);

    /// @brief Turn on the motor
    /// @param buf Output buffer to write the command to
    void create_cmd_motor_on(uint8_t buf[8]);

    /// @brief Stop the motor but dont clear it's state
    /// @param buf Output buffer to write the command to
    void create_cmd_motor_stop(uint8_t buf[8]);

    /// @brief Set the motor torque
    /// @param buf Output buffer to write the command to
    /// @param torque The torque value in range [-2048, 2048] corresponds to [-33A, 33A]
    void create_cmd_torque_control(uint8_t buf[8], int16_t torque);

    /// @brief Set the motor speed
    /// @param buf Output buffer to write the command to
    /// @param speed The speed value in 0.01dps / LSB
    void create_cmd_speed_control(uint8_t buf[8], int32_t speed);

    /// @brief Set the motor angle
    /// @param buf Output buffer to write the command to
    /// @param angle The angle value in 0.01deg / LSB
    // TODO: should angle be signed?
    void create_cmd_multi_angle_control(uint8_t buf[8], int32_t angle);

    /// @brief Set the motor angle with speed limit
    /// @param buf Output buffer to write the command to
    /// @param angle The angle value in 0.01deg / LSB
    /// @param speed_limit The speed limit in 1dps / LSB
    void create_cmd_multi_angle_control_speed_limited(uint8_t buf[8], int32_t angle, uint16_t speed_limit);

    /// @brief Set the motor angle with spin direction
    /// @param buf Output buffer to write the command to
    /// @param angle The angle value in 0.01deg / LSB
    /// @param spin_direction The spin direction: 0 is clockwise, 1 is counter clockwise
    void create_cmd_angle_control(uint8_t buf[8], uint32_t angle, uint8_t spin_direction);

    /// @brief Set the motor angle with spin direction and speed limit
    /// @param buf Output buffer to write the command to
    /// @param angle The angle value in 0.01deg / LSB
    /// @param spin_direction The spin direction: 0 is clockwise, 1 is counter clockwise
    /// @param speed_limit The speed limit in 1dps / LSB
    void create_cmd_angle_control_speed_limited(uint8_t buf[8], uint32_t angle, uint8_t spin_direction, uint16_t speed_limit);

    /// @brief Set the motor angle by increment
    /// @param buf Output buffer to write the command to
    /// @param angle_increment The angle increment in 0.01deg / LSB
    void create_cmd_angle_increment_control(uint8_t buf[8], int32_t angle_increment);
    
    /// @brief Set the motor angle by increment with speed limit
    /// @param buf Output buffer to write the command to
    /// @param angle_increment The angle increment in 0.01deg / LSB
    /// @param speed_limit The speed limit in 1dps / LSB
    void create_cmd_angle_increment_control_speed_limited(uint8_t buf[8], int32_t angle_increment, uint16_t speed_limit);
    
    /// @brief Read PID values
    /// @param buf Output buffer to write the command to
    void create_cmd_read_pid(uint8_t buf[8]);

    /// @brief Write PID values to RAM
    /// @param buf Output buffer to write the command to
    /// @param angle_p The angle P value
    /// @param angle_i The angle I value
    /// @param speed_p The speed P value
    /// @param speed_i The speed I value
    /// @param torque_p The torque P value
    /// @param torque_i The torque I value
    void create_cmd_write_pid(uint8_t buf[8], uint8_t angle_p, uint8_t angle_i, uint8_t speed_p, uint8_t speed_i, uint8_t torque_p, uint8_t torque_i);

    /// @brief Write PID values to ROM
    /// @param buf Output buffer to write the command to
    /// @param angle_p The angle P value
    /// @param angle_i The angle I value
    /// @param speed_p The speed P value
    /// @param speed_i The speed I value
    /// @param torque_p The torque P value
    /// @param torque_i The torque I value
    void create_cmd_write_pid_rom(uint8_t buf[8], uint8_t angle_p, uint8_t angle_i, uint8_t speed_p, uint8_t speed_i, uint8_t torque_p, uint8_t torque_i);

    /// @brief Read acceleration
    /// @param buf Output buffer to write the command to
    void create_cmd_read_acceleration(uint8_t buf[8]);

    /// @brief Write acceleration to RAM
    /// @param buf Output buffer to write the command to
    /// @param acceleration The acceleration value in 1dps / LSB
    void create_cmd_write_acceleration(uint8_t buf[8], int32_t acceleration);

    /// @brief Read the current encoder position
    /// @param buf Output buffer to write the command to
    void create_cmd_read_encoder(uint8_t buf[8]);

    /// @brief Write the encoder value to ROM as the motor zero point
    /// @param buf Output buffer to write the command to
    /// @param encoder_offset The encoder offset value
    void create_cmd_write_encoder_zero(uint8_t buf[8], uint16_t encoder_offset);

    /// @brief Write the current motor position to ROM as the motor zero point
    /// @param buf Output buffer to write the command to
    void create_cmd_write_position_as_zero(uint8_t buf[8]);

    /// @brief Read the multi angle value
    /// @param buf Output buffer to write the command to
    void create_cmd_read_multi_angle(uint8_t buf[8]);

    /// @brief Read the motor angle
    /// @param buf Output buffer to write the command to
    void create_cmd_read_angle(uint8_t buf[8]);

    /// @brief Read the motor state 1 and error state
    /// @param buf Output buffer to write the command to
    void create_cmd_read_state_1(uint8_t buf[8]);

    /// @brief Clear the motor error state
    /// @param buf Output buffer to write the command to
    void create_cmd_clear_error(uint8_t buf[8]);

    /// @brief Read the motor state 2
    /// @param buf Output buffer to write the command to
    void create_cmd_read_state_2(uint8_t buf[8]);

    /// @brief Read the motor state 3
    /// @param buf Output buffer to write the command to
    void create_cmd_read_state_3(uint8_t buf[8]);

    // TODO: multi motor commands, will we use them?
    

private:
    /// @brief The base ID for the motor, the true id is this + the motor id
    const uint32_t m_base_id = 0x140;

    /// @brief The maximum torque value, corresponds to 33A
    const int32_t m_max_torque = 2048;
    /// @brief The minimum torque value, corresponds to -33A
    const int32_t m_min_torque = -2048;

    // various special state
    // TODO: should the PID params be unsigned?
    /// @brief The angle P value
    int8_t m_angle_p = 0;

    /// @brief The angle I value
    int8_t m_angle_i = 0;

    /// @brief The speed P value
    int8_t m_speed_p = 0;

    /// @brief The speed I value
    int8_t m_speed_i = 0;

    /// @brief The torque P value
    int8_t m_torque_p = 0;

    /// @brief The torque I value
    int8_t m_torque_i = 0;

    /// @brief The acceleration value in 1dps / LSB
    int32_t m_acceleration = 0;

    /// @brief The encoder value (raw - offset)
    uint16_t m_encoder = 0;

    /// @brief The encoder raw value
    uint16_t m_encoder_raw = 0;

    /// @brief The encoder offset value
    uint16_t m_encoder_offset = 0;

    /// @brief The multi motor angle value in 0.01deg / LSB. This is the cumulative angle from all motors set in multi mode
    int64_t m_multi_motor_mode_angle = 0;

    /// @brief The single motor angle value in 0.01deg / LSB, wraps to 0 after (36000 - 1)
    uint32_t m_single_motor_mode_angle = 0;

    /// @brief The motor voltage in 0.1V / LSB
    uint16_t m_voltage = 0;

    /// @brief The error state of the motor
    struct {
        /// @brief 0: normal, 1: under voltage error
        uint8_t under_voltage : 1;
        // reserved
        uint8_t : 2;
        /// @brief 0: normal, 1: over temperature error
        uint8_t over_temperature : 1;
        // reserved
        uint8_t : 4;
    } m_error_state;

    /// @brief A phase current in 1A / 64LSB
    int16_t m_a_phase_current = 0;
    
    /// @brief B phase current in 1A / 64LSB
    int16_t m_b_phase_current = 0;
    
    /// @brief C phase current in 1A / 64LSB
    int16_t m_c_phase_current = 0;

    // Command command byte definitions

    static constexpr uint8_t CMD_MOTOR_OFF = 0x80;
    static constexpr uint8_t CMD_MOTOR_ON = 0x88;
    static constexpr uint8_t CMD_MOTOR_STOP = 0x81;
    static constexpr uint8_t CMD_TORQUE_CONTROL = 0xA1;
    static constexpr uint8_t CMD_SPEED_CONTROL = 0xA2;
    static constexpr uint8_t CMD_MULTI_ANGLE_CONTROL = 0xA3;
    static constexpr uint8_t CMD_MULTI_ANGLE_CONTROL_SPEED_LIMITED = 0xA4;
    static constexpr uint8_t CMD_ANGLE_CONTROL = 0xA5;
    static constexpr uint8_t CMD_ANGLE_CONTROL_SPEED_LIMITED = 0xA6;
    static constexpr uint8_t CMD_ANGLE_INCREMENT_CONTROL = 0xA7;
    static constexpr uint8_t CMD_ANGLE_INCREMENT_CONTROL_SPEED_LIMITED = 0xA8;
    static constexpr uint8_t CMD_READ_PID = 0x30;
    static constexpr uint8_t CMD_WRITE_PID = 0x31;
    static constexpr uint8_t CMD_WRITE_PID_ROM = 0x32;
    static constexpr uint8_t CMD_READ_ACCELERATION = 0x33;
    static constexpr uint8_t CMD_WRITE_ACCELERATION = 0x34;
    static constexpr uint8_t CMD_READ_ENCODER = 0x90;
    static constexpr uint8_t CMD_WRITE_ENCODER_ZERO = 0x91;
    static constexpr uint8_t CMD_WRITE_POSITION_AS_ZERO = 0x19;
    static constexpr uint8_t CMD_READ_MULTI_ANGLE = 0x92;
    static constexpr uint8_t CMD_READ_ANGLE = 0x94;
    static constexpr uint8_t CMD_READ_STATE_1 = 0x9A;
    static constexpr uint8_t CMD_CLEAR_ERROR = 0x9B;
    static constexpr uint8_t CMD_READ_STATE_2 = 0x9C;
    static constexpr uint8_t CMD_READ_STATE_3 = 0x9D;

};

#endif // MG8016E_I6_DRIVER_HPP
