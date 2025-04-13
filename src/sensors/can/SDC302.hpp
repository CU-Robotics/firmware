#pragma once

#include "motor.hpp"
#include <cstdint>
// Data of the GIM6010-6 motor https://steadywin.cn/en/pd.jsp?id=12#_jcp=3_3
/// @brief SDC302 motor controller class
class SDC302 : public Motor {
public:
    /// @brief Deleted default constructor, must explicitly construct this object. Incomplete objects are not allowed
    SDC302() = delete;

    /// @brief Main constructor. Defines the controller type, global ID, id, can bus, and motor type
    /// @param gid The global ID, not the per-bus motor ID
    /// @param id The per-bus motor ID. This is 1-indexed
    /// @param bus_id The CAN bus index/ID
    /// @param motor_type The motor type (not used for SDC302, do not specify)
    SDC302(uint32_t gid, uint32_t id, uint8_t bus_id, MotorType motor_type)
        : Motor(MotorControllerType::SDC302, gid, id, bus_id, motor_type) {
    }

    /// @brief Destructor, does nothing
    ~SDC302() override { }

public:
    /// @brief Initialize the motor by verifying it is on
    void init() override;

    /// @brief Common read command. Fills given message if successful
    /// @param msg The message buffer to fill data into
    /// @return 0 on failure, 1 on success
    int read(CAN_message_t& msg) override;

    /// @brief Common write command. This fills the given message if successful. This is used to compile data to be sent over the CAN line
    /// @param msg The message buffer to fill data into
    /// @return The index in the buffer where the motor data was written. The SDC302 driver will always write 8 bytes so this will always return 0
    /// @note Does not issue a CAN command over the bus
    /// @note Return value is mostly useless for this motor, it will always be 0
    int write(CAN_message_t& msg) const override;

    /// @brief Zero the motor. This is a safety function to ensure the motor is not actively driving
    /// @note Does not issue a CAN command over the bus
    void zero_motor() override;

    /// @brief Write motor torque given a normalized value
    /// @param torque A value between [-1, 1] representing the torque range of [-18Nm, 18Nm]
    void write_motor_torque(float torque) override;

    /// @brief Print the current state of the motor
    void print_state() const override;

public:
    /// @brief Turn on the motor
    void write_motor_on();

    /// @brief Zero the motor's encoder
    void write_motor_zero();

    /// @brief Turn off the motor
    void write_motor_off();

private:
    /// @brief Create a motor on command
    /// @param msg The message to fill in
    void create_cmd_motor_on(CAN_message_t& msg);

    /// @brief Create a motor zero command
    /// @param msg The message to fill in
    void create_cmd_motor_zero(CAN_message_t& msg);

    /// @brief Create a motor off command
    /// @param msg The message to fill in
    void create_cmd_motor_off(CAN_message_t& msg);

    /// @brief Create a motor position, velocity, and torque command
    /// @param msg The message to fill in
    /// @param position The new position in rad [-4pi, 4pi]
    /// @param velocity The new velocity in rad/s [-30, 30]
    /// @param kp The proportional gain in Nm/rad [0, 500]
    /// @param kd The derivative gain in (Nm*s)/rad [0, 5]
    /// @param torque_ff The feedforward torque in Nm [-18, 18]
    void create_cmd_control(CAN_message_t& msg, int16_t position, int16_t velocity, int16_t kp, int16_t kd, int16_t torque_ff);

private:
    /// @brief The motor's maximum torque in Nm
    float m_max_torque = 18.0f;

};