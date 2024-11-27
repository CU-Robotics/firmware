#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include "motor.hpp"

/// @brief The maximum number of motors that can be connected to the system. Derrived from the max motors per bus assuming bus 3 only holds non-RM motors
constexpr uint32_t CAN_MAX_MOTORS = 8u + 8u + 32u;

/// @brief The number of CAN busses that can be used
constexpr uint32_t CAN_NUM_BUSSES = 3u;

/// @brief CAN bus 1 ID
constexpr uint32_t CAN_1 = 0u;
/// @brief CAN bus 2 ID
constexpr uint32_t CAN_2 = 1u;
/// @brief CAN bus 3 ID
constexpr uint32_t CAN_3 = 2u;

// Because of how volatile FlexCAN_T4 objects are, there must only ever exist one instance of this class
// I'm deleting any copy and assignment constructors that may act to create a new instance

class CANManager {
public:
    CANManager();

    /// @brief Deleted copy and assignment constructors. This class should only ever have one instance
    /// @param copy The CANManager object to copy
    CANManager(const CANManager& copy) = delete;
    /// @brief Deleted copy and assignment constructors. This class should only ever have one instance
    /// @param copy The CANManager object to copy
    /// @return CANManager& The reference to this object
    CANManager& operator=(const CANManager&) = delete;

    /// @brief Destructor, cleans up motor array
    ~CANManager();

public:
    /// @brief Initialize the CAN buses and motor array
    /// @note This can be called multiple times in the event of hot reloading
    void init();

    /// @brief Dynamically create the motor objects based on config data
    /// @param motor_info Motor info array from the config yaml. 2D array holding information in the form: CAN_MAX_MOTORS * [motor_controller_type, per_bus_motor_id, bus_id]
    void configure(float motor_info[CAN_MAX_MOTORS][3]);

    void read();

    void write();

    /// @brief Write a torque command to a specific motor given it's global ID
    /// @param motor_gid The global ID of the motor to write to
    /// @param torque The normalized torque value to write to the motor in the range [-1, 1]
    /// @note This does not issue a CAN command over the bus
    void write_motor_torque(uint32_t motor_gid, float torque);

private:


private:
    /// @brief CAN bus 1
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> m_can1;
    /// @brief CAN bus 2
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> m_can2;
    /// @brief CAN bus 3
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> m_can3;

    /// @brief Array of generic motor objects
    Motor* m_motors[CAN_MAX_MOTORS] = { nullptr };

};

#endif // CAN_MANAGER_HPP