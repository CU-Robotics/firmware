#ifndef CAN_MOTOR_HPP
#define CAN_MOTOR_HPP

#include <Arduino.h>

#include <FlexCAN_T4.h>

/// @brief Defines the motor controller types. Enum values chosen based on the yaml config specification
enum class MotorControllerType {
    NULL_MOTOR_CONTROLLER_TYPE = -1,
    C610_CONTROLLER,
    C620_CONTROLLER,
    MG8016_CONTROLLER,
};

/// @brief Unified motor state
struct MotorState {
    /// @brief Motor specific torque output. Unit is normalized to [-1, 1] representing the signed percentage of max torque
    float torque = 0;
    /// @brief Rotational speed of the motor in rad/s, signed
    float speed = 0;
    /// @brief Motor specific position output. The unit is dependent on the motor but it is normally in a range corresponding to it's encoder
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

    /// @brief Main constructor. Defines the motor and controller type, global ID, id, and can bus
    /// @param controller_type The motor controller type 
    /// @param gid The global ID, not the per-bus motor ID
    /// @param id The per-bus motor ID. This is 1-indexed 
    /// @param bus The CAN bus index/ID
    Motor(MotorControllerType controller_type, uint32_t gid, uint32_t id, uint8_t bus)
        : m_controller_type(controller_type), m_gid(gid), m_id(id), m_bus_id(bus) {}

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
    virtual void write_motor_torque(float torque) = 0;

    /// @brief Print the current state of the motor
    virtual void print_state() const = 0;

public:
    /// @brief Get the motor controller type
    /// @return The motor controller type
    inline MotorControllerType get_controller_type() const { return m_controller_type; }

    /// @brief Get the global ID
    /// @return The unique global ID
    inline uint32_t get_global_id() const { return m_gid; }

    /// @brief Get the per-bus motor ID
    /// @return The CAN motor ID
    inline uint32_t get_id() const { return m_id; }

    /// @brief Get the bus ID
    /// @return The bus ID
    inline uint32_t get_bus_id() const { return m_bus_id; }

    /// @brief Get the current state of the motor
    /// @return The current state of the motor
    inline MotorState get_state() const { return m_state; }

protected:
    /// @brief What controller this motor uses
    MotorControllerType m_controller_type = MotorControllerType::NULL_MOTOR_CONTROLLER_TYPE;

    /// @brief The unique global motor ID for this motor. This is not it's per-bus ID
    uint32_t m_gid = 0;

    /// @brief The per-bus CAN id for this motor. This is not it's global ID from config
    uint32_t m_id = 0;

    /// @brief ID of the CAN bus
    uint8_t m_bus_id = 0;

    /// @brief The output CAN frame. To be sent to the motor
    CAN_message_t m_output;

    /// @brief The last received CAN frame. Received from the motor
    CAN_message_t m_input;

    /// @brief The current state of the motor
    MotorState m_state;

};

#endif // CAN_MOTOR_HPP