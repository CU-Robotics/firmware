#ifndef C620_DRIVER_HPP
#define C620_DRIVER_HPP

#include "motor.hpp"

/// @brief C620 controller driver. This manages generating CAN output messages and processing incomming CAN messages into a state array.
/// @brief Datasheet: https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf
/// @note It's construction should be closely managed. This object exists only to be managed by CANManager.
class C620 : public Motor {
public:
    /// @brief Deleted default constructor, must explicitly construct this object. Incomplete objects are not allowed
    C620() = delete;

    /// @brief Main constructor. Defines the controller type, global ID, id, and can bus
    /// @param gid The global ID, not the per-bus motor ID
    /// @param id The per-bus motor ID. This is 1-indexed
    /// @param bus_id The CAN bus index/ID
    C620(uint32_t gid, uint32_t id, uint8_t bus_id)
        : Motor(C620_CONTROLLER, gid, id, bus_id) {
    }

    /// @brief Destructor, does nothing
    ~C620() override { }

public:
    /// @brief Initialize the motor by zeroing it
    void init();

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
    /// @param torque A value between [-1, 1] representing the torque range of [-20A, 20A]
    void write_motor_torque(float torque) override;

    /// @brief Print the current state of the motor
    void print_state() const override;

private:


private:
    /// @brief The base ID of the motor
    uint32_t m_base_id = 0x200;

    /// @brief The maximum torque value
    const int32_t m_max_torque = 16384;
    /// @brief The minimum torque value
    const int32_t m_min_torque = -16384;

};

#endif // C620_DRIVER_HPP
