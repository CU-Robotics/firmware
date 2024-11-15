#ifndef CAN_MOTOR_HPP
#define CAN_MOTOR_HPP

#include <Arduino.h>

#include <FlexCAN_T4.h>

/// @brief Defines the motor types
enum MotorType {
    GIM3505_8 = 0,
    GIM4310_36,
    GIM6010_8,
    GIM8108_9,
    MG8016E_I6V3,
    M3508,
    M2006,
    
    // add types above this comment
    NUM_MOTOR_TYPES,
    NULL_MOTOR_TYPE
};

/// @brief Defines the motor controller types
enum MotorControllerType {
    C610 = 0,
    C620,
    INTERNAL,
    
    // add types above this comment
    NUM_MOTOR_CONTROLLER_TYPES,
    NULL_MOTOR_CONTROLLER_TYPE
};

/// @brief An abstract class holding common information for individual CAN-capable motors
class Motor {
public:
    Motor() = delete;
    Motor(MotorType type, MotorControllerType controller_type, uint32_t gid, uint32_t id, uint8_t bus)
        : m_motor_type(type), m_controller_type(controller_type), m_gid(gid), m_id(id), m_bus_id(bus) {}

    virtual ~Motor() = 0;

public:
    /// @brief Common read command. Fills given message if successful
    /// @param msg The message buffer to fill data into
    /// @return 0 on failure, 1 on success
    virtual int read(CAN_message_t& msg) = 0;

    /// @brief Common write command. This fills the given message if successful. This is used to compile data to be sent over the CAN line
    /// @param msg The message buffer to fill data into
    /// @return 0 on failure, 1 on success
    /// @note Does not issue a CAN command over the bus
    virtual int write(CAN_message_t& msg) = 0;

    // TODO: how?
    /// @brief Generic write motor function handling only torque. This is the only common interface of all the motors we use
    /// @param torque The torque value
    virtual void write_motor_torque(int32_t torque) = 0;

public:
    /// @brief Get the motor type
    /// @return The motor type
    inline MotorType get_motor_type() const { return m_motor_type; }

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

protected:
    // TODO: private functions?

protected:
    /// @brief What type of motor it is
    MotorType m_motor_type = NULL_MOTOR_TYPE;

    /// @brief What controller this motor uses
    MotorControllerType m_controller_type = NULL_MOTOR_CONTROLLER_TYPE;

    /// @brief The unique global motor ID for this motor. This is not it's per-bus ID
    uint32_t m_gid = 0;

    /// @brief The per-bus CAN id for this motor. This is not it's global ID from config
    uint32_t m_id = 0;

    /// @brief ID of the CAN bus
    uint8_t m_bus_id = 0;

    // TODO: more common information?
};

#endif // CAN_MOTOR_HPP