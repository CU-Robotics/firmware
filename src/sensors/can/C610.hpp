#ifndef C610_DRIVER_HPP
#define C610_DRIVER_HPP

#include "motor.hpp"

/// @brief C610 controller driver. This manages generating CAN output messages and processing incomming CAN messages into a state array.
/// @brief Datasheet: https://rm-static.djicdn.com/tem/17348/RoboMaster%20C610%20Brushless%20DC%20Motor%20Speed%20Controller%20User%20Guide.pdf
/// @note Motors must be explicitly constructed to avoid uninitialized parameters. Motors exist to be managed by CANManager.
class C610 : public Motor {
public:
    /// @brief Deleted default constructor, must explicitly construct this object. Incomplete objects are not allowed
    C610() = delete;

    /// @brief Main constructor. Defines the controller type, global ID, id, and can bus
    /// @param gid The global ID, not the per-bus motor ID
    /// @param id The per-bus motor ID. This is 1-indexed
    /// @param bus_id The CAN bus index/ID
    /// @param motor_type The motor type, defaults to NULL_MOTOR_TYPE if not specified (not used for C610)
    C610(uint32_t gid, uint32_t id, uint8_t bus_id, MotorType motor_type)
        : Motor(MotorControllerType::C610, gid, id, bus_id, motor_type) {
        m_base_id = 0x200;
    }

    /// @brief Destructor, does nothing
    ~C610() override { }

public:
    /// @brief Initialize the motor by zeroing it
    void init() override;

    /// @brief Common read command. Fills given message if successful
    /// @param msg The message buffer to fill data into
    /// @return 0 on failure, 1 on success
    int read(CAN_message_t& msg) override;

    /// @brief Common write command. This fills the given message if successful. This is used to compile data to be sent over the CAN line
    /// @param msg The message buffer to fill data into
    /// @return The index in the buffer where the motor data was written
    /// @note Does not issue a CAN command over the bus
    int write(CAN_message_t& msg) const override;

    /// @brief Zero the motor. This is a safety function to ensure the motor is not actively driving
    /// @note Does not issue a CAN command over the bus
    void zero_motor() override;

    /// @brief Write motor torque given a normalized value
    /// @param torque A value between [-1, 1] representing the torque range of [-10A, 10A]
    void write_motor_torque(float torque) override;

private:


private:
    /// @brief The maximum torque value
    const int32_t m_max_torque = 10000;
    /// @brief The minimum torque value
    const int32_t m_min_torque = -10000;

};

#endif // C610_DRIVER_HPP