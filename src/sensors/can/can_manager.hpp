#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include "motor.hpp"


#include <map>
#include <memory>
/// @brief Debug flag to enable verbose logging. This can get quite noisy so it is off by default. Critical errors are still printed regardless of flag
// #define CAN_MANAGER_DEBUG

/// @brief The maximum number of motors that can be connected to a single bus
constexpr uint32_t CAN_MAX_MOTORS_PER_BUS = 8u;

/// @brief The number of CAN busses that can be used
constexpr uint32_t CAN_NUM_BUSSES = 3u;

/// @brief The maximum number of motors that can be connected to the system. Derrived from the max motors per bus assuming bus 3 only holds non-RM motors
constexpr uint32_t CAN_MAX_MOTORS = CAN_NUM_BUSSES * CAN_MAX_MOTORS_PER_BUS;

/// @brief CAN bus 1 ID
constexpr uint32_t CAN_1 = 0u;
/// @brief CAN bus 2 ID
constexpr uint32_t CAN_2 = 1u;
/// @brief CAN bus 3 ID
constexpr uint32_t CAN_3 = 2u;

// Because of how volatile FlexCAN_T4 objects are, there must only ever exist one instance of this class
// I'm deleting any copy and assignment constructors that may act to create a new instance

/// @brief The overall manager for the CAN busses and motors
/// @note The construction of FlexCAN_T4 objects in of themselves alters processsor state. 
/// When these are constructed incorrectly, it will exibit the folowing behavior: 
///     reads are fine, 
///     the first write will not hang, the processor will lock up some small (seemingly random) amount of time after the write call. 
/// Sometimes this even allows lines of code after the first write call to be ran (like prints). 
/// Examples of incorrect construction of FlexCAN_T4 objects include: 
///     construction in function scope (including main), 
///     construction via new (likely, although this is never tested. It is an active issue on FlexCAN_T4's github though), 
///     and construction via copy/assignment. 
/// This is why we heavily manage the construction of the CANManager
class CANManager {
public:
    /// @brief Default constructor. FlexCAN moment for forcing the definition to be in the cpp
    CANManager();

    /// @brief Deleted copy and assignment constructors. This class should only ever have one instance
    /// @param copy The CANManager object to copy
    CANManager(const CANManager& copy) = delete;
    /// @brief Deleted copy and assignment constructors. This class should only ever have one instance
    /// @param copy The CANManager object to copy
    /// @return CANManager& The reference to this object
    CANManager& operator=(const CANManager& copy) = delete;

    /// @brief Destructor, cleans up motor array
    ~CANManager();

public:
    /// @brief Initialize the CAN buses and motor map
    /// @note This can be called multiple times in the event of hot reloading
    void init(const std::vector<NewConfig::Motor>& motor_configs);

    /// @brief Configure a motor and add it to the motor map
    /// @param motor_info motor configuration data
    void configure_motor(const NewConfig::Motor& motor_config);

    /// @brief Read data from all busses and distribute them to the correct motors
    void read();

    /// @brief Issue write commands to all motors on each bus
    /// @note This issues a CAN command over the bus
    void write();

    void send_to_comms();

    /// @brief Issue zero torque commands to all motors
    /// @note This immediately issues a CAN command over the bus
    void issue_safety_mode();

    /// @brief Write a torque command to a specific motor by name
    /// @param motor_name The name of the motor to write to
    /// @param torque The torque command to write to the motor
    void write_motor_torque_by_name(NewConfig::MotorName motor_name, float torque);

    /// @brief Print the state of all motors
    void print_state();

    /// @brief Print the state of a specific motor
    /// @param motor_name The name of the motor to print the state of
    void print_motor_state_by_name(NewConfig::MotorName motor_name);

    /// @brief Get the underlying motor object by name
    /// @param motor_name The name of the motor to get
    /// @return The motor object if it exists, std::nullopt if it does not
    /// @note You can use the motor's get_type to determine the type of motor and dynamic_cast to the correct motor type
    Motor* get_motor_by_name(NewConfig::MotorName motor_name) const;

    /// @brief Get the state of a specific motor by name
    /// @param motor_name The name of the motor to get the state of
    /// @return The state of the motor if the motor exists, std::nullopt if it does not
    std::optional<MotorState> get_motor_state_by_name(NewConfig::MotorName motor_name) const ;

private:
    /// @brief Verify that all motors are online and ready
    void init_motors();

    /// @brief Iterates through all the motors and tries to give the message to the correct one
    /// @param msg The message to distribute
    /// @return The motor that successfully read the message or NewConfig::MotorName::UnsetMotorName if no motor read the message
    NewConfig::MotorName distribute_msg(CAN_message_t& msg);

private:
    /// @brief CAN bus 1
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> m_can1;
    /// @brief CAN bus 2
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> m_can2;
    /// @brief CAN bus 3
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> m_can3;

    /// @brief Array of CAN bus objects for easier access
    FlexCAN_T4_Base* m_busses[CAN_NUM_BUSSES] = { &m_can1, &m_can2, &m_can3 };

    /// @brief Map of motor name to allocated motor object pointer
    std::map<NewConfig::MotorName, std::unique_ptr<Motor>> m_motor_name_map;

    /// @brief The timeout for motor initialization in milliseconds. Most motors respond within 1-2 ms
    uint32_t m_motor_init_timeout = 250u;

};

#endif // CAN_MANAGER_HPP
