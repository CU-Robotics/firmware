#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include "motor.hpp"

/// @brief The maximum number of motors that can be connected to the system. Derrived from the max motors per bus assuming bus 3 only holds non-RM motors
constexpr uint32_t CAN_MAX_MOTORS = 8 + 8 + 32;

// Because of how volatile FlexCAN_T4 objects are, there must only ever exist one instance of this class
// I'm deleting any copy and assignment constructors that may act to create a new instance

class CanManager {
public:
    CanManager() = default;

public:
    void init();

    /// @brief Dynamically create the motor objects based on config data
    /// @param motor_info Motor info array from the config yaml. 2D array holding information in the form: CAN_MAX_MOTORS * [motor_controller_type, per_bus_motor_id, bus_id]
    void configure(float motor_info[CAN_MAX_MOTORS][3]);

    void read();

    void write();

    void write_motor_torque(uint32_t motor_gid, float torque);

private:


private:
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> m_can1;
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> m_can2;
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> m_can3;

    Motor* m_motors[CAN_MAX_MOTORS] = { nullptr };

};

#endif // CAN_MANAGER_HPP