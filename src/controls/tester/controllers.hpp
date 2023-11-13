#ifndef CONTROLLERS_HPP
#define CONTROLLERS_HPP

#include "rm_CAN.hpp"
#include "dr16.hpp"

constexpr uint16_t CAN_1 = 0;             // CAN 1 (indexable value)
constexpr uint16_t CAN_2 = 1;             // CAN 2 (indexable value)

constexpr uint16_t NUM_CANS = 2;          // 2 cans per robot
constexpr uint16_t NUM_MOTORS = 8;        // 8 motors per can

constexpr uint16_t MAX_WRITE = 4000;      // Safety measure to limit how much torque can be applied to motors

class Controllers
{
public:
    /// @brief Constructor. Calls init for can and remote
    Controllers();

    /// @brief Updates the motors with respect to input. Also includes safety code
    void update();

    
private:
    /// @brief Used to read and write values to each can
    rm_CAN can;

    /// @brief Used to get data from the controller
    DR16 remote;

    /// @brief stores the target speeds for each of the motors
    int setSpeeds[NUM_CANS][NUM_MOTORS];

    /// @brief sets all set speeds of every motor to 0
    void zero_speeds();

    /// @brief Writes to motors. Uses PID to set the speeds of motors
    void update_speeds();

    /// @brief sets the target speed of a motor
    /// @param _canNum The can the motor is on
    /// @param _mtrNum the motor number
    /// @param val the value you want to set
    void set_mtr_speed(int _canNum, int _mtrNum, int val);
};

#endif // CONTROLLERS_HPP