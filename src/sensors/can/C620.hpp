#ifndef C620_DRIVER_HPP
#define C620_DRIVER_HPP

#include "motor.hpp"

// C620 User Guide
// https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf

template <typename CAN_BUS>
class C620 : public Motor {
public:
    C620() = delete;

    C620(MotorType type, uint32_t gid, uint32_t id, uint8_t bus_id, CAN_BUS* bus)
        : Motor(type, MotorControllerType::C620_CONTROLLER, gid, id, bus_id), m_can_bus(bus) {
    }

    C620(const C620& copy) = delete;
    C620& operator=(const C620& copy) = delete;

    ~C620() override { }

public:
    int read(CAN_message_t& msg) override;

    int write(CAN_message_t& msg) override;

    void write_motor_torque(float torque) override;

private:


private:
    CAN_BUS* m_can_bus = nullptr;

    const int32_t m_max_torque = 16384;
    const int32_t m_min_torque = -16384;

};

#endif // C620_DRIVER_HPP
