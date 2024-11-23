#ifndef C620_DRIVER_HPP
#define C620_DRIVER_HPP

#include "motor.hpp"

// C620 Data Sheet
// https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf

template <typename CAN_BUS>
class C620 : public Motor {
public:
    /// @brief Deleted default constructor, must explicitly construct this object
    C620() = delete;

    /// @brief Main constructor. Defines the motor and controller type, global ID, id, and can bus
    /// @param type The underlying motor type
    /// @param gid The global ID, not the per-bus motor ID
    /// @param id The per-bus motor ID. This is 1-indexed
    /// @param bus_id The CAN bus index/ID
    /// @param bus The CAN bus object
    C620(MotorType type, uint32_t gid, uint32_t id, uint8_t bus_id, CAN_BUS* bus)
        : Motor(type, MotorControllerType::C620_CONTROLLER, gid, id, bus_id), m_can_bus(bus) {
    }

    /// @brief Deleted copy constructor
    /// @param copy copy
    C620(const C620& copy) = delete;
    /// @brief Deleted copy assignment operator
    /// @param copy copy
    /// @return C620&
    C620& operator=(const C620& copy) = delete;

    /// @brief Virtual destructor
    ~C620() override { }

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
    /// @param torque A value between [-1, 1] representing the torque range of [-20A, 20A]
    void write_motor_torque(float torque) override;

    /// @brief Print the current state of the motor
    void print_state() override;

private:


private:
    /// @brief The CAN bus object
    CAN_BUS* m_can_bus = nullptr;

    /// @brief The maximum torque value
    const int32_t m_max_torque = 16384;
    /// @brief The minimum torque value
    const int32_t m_min_torque = -16384;

};

#endif // C620_DRIVER_HPP
