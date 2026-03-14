#ifndef CAN_MOTOR_HPP
#define CAN_MOTOR_HPP

#include <Arduino.h>
#include "comms/config_data/motor.hpp"

#include <FlexCAN_T4.h>
#include <cstdint>

/// @brief Unified motor state
struct MotorState {
    /// @brief Motor specific torque output. Unit is normalized to [-1, 1] representing the signed percentage of max torque
    float torque = 0;
    /// @brief Rotational speed of the motor in rad/s, signed
    float speed = 0;
    /// @brief Motor specific position output. The unit is dependent on the motor but it is normally in a range corresponding to it's encoder
    // TODO: This should be in radians and be a float
    uint16_t position = 0;
    /// @brief Temperature of the motor in degrees Celsius
    /// @note This is signed but it shouldn't matter since normal temperatures wouldn't overflow to negative
    int8_t temperature = 0;
};

/// @brief An abstract class holding common information for individual CAN-capable motors
/// @note Motors must be explicitly constructed to avoid uninitialized parameters. Motors exist to be managed by CANManager.
class Motor {
public:
    /// @brief Deleted default constructor. Must explicitly construct this object
    Motor() = delete;

    /// @brief Main constructor. 
    /// @param motor_config The configuration struct for this motor
    Motor(Cfg::Motor motor_config)
         : motor_config(motor_config), m_physical_id(motor_config.physical_id), m_bus_id(motor_config.physical_bus) { }

/// @brief Virtual destructor
    virtual ~Motor() { }

public:
    /// @brief Initialize the motor and perform any startup commands
    virtual void init() = 0;

    /// @brief Common read command. Fills given message if successful
    /// @param msg The message buffer to fill data into
    /// @return 0 on failure, 1 on success
    virtual int read(CAN_message_t& msg) = 0;

    /// @brief Common write command. This fills the given message if successful. This is used to compile data to be sent over the CAN line
    /// @param msg The message buffer to fill data into
    /// @return The index into msg buf where the motor data was written
    /// @note Does not issue a CAN command over the bus
    virtual int write(CAN_message_t& msg) const = 0;

    /// @brief Zero the motor. This is a safety function to ensure the motor is not actively driving
    /// @note Does not issue a CAN command over the bus
    virtual void zero_motor() = 0;  // this is virtual because it might differ between motors

    /// @brief Generic write motor function handling only torque. This is the only common interface of all the motors we use
    /// @param torque The torque value between [-1, 1]
    /// @note This does not issue a CAN command over the bus
    void write_motor_torque(float torque) {
        commanded_motor_torque = torque;
        execute_motor_torque_command(torque);
    }

    /// @brief Get the last commanded motor torque. This is used to send to comms so we can log the commanded torque and not just the actual torque
    /// @return The last commanded motor torque
    float get_commanded_motor_torque() const {
        return commanded_motor_torque;
    }

    /// @brief Print the current state of the motor
    virtual void print_state() const {
        Serial.printf("Name: %u\tBus: %u\tID: %u\tTemp: %.2dc\tTorque: % 4.3f%%\tSpeed: % 6.2frad/s\tPos: %5.5d?\n", static_cast<uint32_t>(motor_config.motor_name), motor_config.physical_bus, motor_config.physical_id, m_state.temperature, m_state.torque, m_state.speed, m_state.position);
    }

public:
    /// @brief Get the motor controller type
    /// @return The motor controller type
    inline Cfg::MotorControllerType get_controller_type() const { return motor_config.motor_controller_type; }

    /// @brief Get the motor type
    /// @return The motor type
    inline Cfg::MotorType get_motor_type() const { return motor_config.motor_type; }

    /// @brief Get the per-bus motor ID
    /// @return The CAN motor ID
    inline uint32_t get_id() const { return motor_config.physical_id; }

    /// @brief Get the configured motor name
    /// @return The motor name
    inline Cfg::MotorName get_name() const { return motor_config.motor_name; }

    /// @brief Get the bus ID
    /// @return The bus ID
    inline uint32_t get_bus_id() const { return motor_config.physical_bus; }
    /// @brief Get the current state of the motor
    /// @return The current state of the motor
    inline MotorState get_state() const { return m_state; }

protected:
    /// @brief Check if the message is for this motor
    /// @param msg The message to check
    /// @return True if the message is for this motor, false otherwise
    inline virtual bool check_msg_id(const CAN_message_t& msg) const {
        // early return if msg ID does not match
        if (msg.id != m_base_id + m_physical_id) {
            return false;
        }

        // early return if the bus does not match
        // msg.bus is 1-indexed
        if ((uint32_t)(msg.bus - 1) != motor_config.physical_bus) {
            return false;
        }

        return true;
    }

    /// @brief Execute the motor torque command. This is the actual implementation of writing the motor torque that differs between motors. This is called by write_motor_torque after updating the commanded_motor_torque variable
    /// @param torque The torque value between [-1, 1]
    virtual void execute_motor_torque_command(float torque) = 0;

protected:
    /// @brief motor configuration info, storing motor type, controller type, bus, ID, and name
    Cfg::Motor motor_config;

    /// @brief The base ID for the motor
    uint32_t m_base_id = 0;

    /// @brief The physical ID of the motor on its bus.
    uint32_t m_physical_id = 0;
    /// @brief The bus ID that the motor is on
    uint32_t m_bus_id = 0;

    /// @brief The output CAN frame. To be sent to the motor
    CAN_message_t m_output;

    /// @brief The last received CAN frame. Received from the motor
    CAN_message_t m_input;

    /// @brief The current state of the motor
    MotorState m_state;

private: 
    /// @brief The last commanded motor torque. This is stored to be sent to comms, it is updated whenever write_motor_torque is called
    float commanded_motor_torque = 0.0f;

};

#endif // CAN_MOTOR_HPP
