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
    C610_CONTROLLER = 0,
    C620_CONTROLLER,
    INTERNAL_CONTROLLER,
    
    // add types above this comment
    NUM_MOTOR_CONTROLLER_TYPES,
    NULL_MOTOR_CONTROLLER_TYPE
};

/// @brief Unified motor state
struct MotorState {
    /// @brief Motor specific torque output. The unit is dependent on the motor but it is normally in a range corresponding to it's ampage
    int16_t torque = 0;
    /// @brief Rotational speed of the motor in rad/s, signed
    float speed = 0;
    /// @brief Motor specific position output. The unit is dependent on the motor but it is normally in a range corresponding to it's encoder
    // TODO: unify this to a single unit
    int16_t position = 0;
    /// @brief Temperature of the motor in degrees Celsius
    int8_t temperature = 0; // TODO: why signed?
};

/// @brief An abstract class holding common information for individual CAN-capable motors
class Motor {
public:
    /// @brief Deleted default constructor. Must explicitly construct this object
    Motor() = delete;

    /// @brief Main constructor. Defines the motor and controller type, global ID, id, and can bus
    /// @param type The physical motor type
    /// @param controller_type The motor controller type 
    /// @param gid The global ID, not the per-bus motor ID
    /// @param id The per-bus motor ID. This is 1-indexed 
    /// @param bus The CAN bus index/ID
    Motor(MotorType type, MotorControllerType controller_type, uint32_t gid, uint32_t id, uint8_t bus)
        : m_motor_type(type), m_controller_type(controller_type), m_gid(gid), m_id(id), m_bus_id(bus) {}

    /// @brief Virtual destructor
    virtual ~Motor() { }

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
    /// @param torque The torque value between [-1, 1]
    virtual void write_motor_torque(float torque) = 0;

    /// @brief Print the current state of the motor
    virtual void print_state() = 0;

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

    /// @brief Get the current state of the motor
    /// @return The current state of the motor
    inline MotorState get_state() const { return m_state; }

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

    /// @brief The output CAN frame. To be sent to the motor
    CAN_message_t m_output;

    /// @brief The last received CAN frame. Received from the motor
    CAN_message_t m_input;

    /// @brief The current state of the motor
    MotorState m_state;

    // TODO: more common information?
};

#endif // CAN_MOTOR_HPP