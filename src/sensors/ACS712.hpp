#ifndef ACS712_CURRENT_SENSOR_HPP
#define ACS712_CURRENT_SENSOR_HPP

#include <Arduino.h>

// Datasheet: https://www.sparkfun.com/datasheets/BreakoutBoards/0712.pdf 

/// @brief The pin to read current from. TEENSY I SENSE on the electrical team pinout
constexpr int ACS712_READ_PIN = 38;

/// @brief Class for reading current from ACS712 sensor
class ACS712 {
public:
    /// @brief Default Constructor. Uses the default pin to read current from
    ACS712() : m_pin(ACS712_READ_PIN) {}

    /// @brief Basic Constructor. Assigns whatever pin to read with
    /// @param pin The pin to read current from
    ACS712(int pin) : m_pin(pin) {};

    /// @brief Reads the current from the ACS712 sensor
    /// @return The current read from the sensor in mA
    float read();
    /// @brief gets the last read current (lowpass filtered) from the sensor
    /// @return float current
    inline float get_current() { return current; }

private:
    /// @brief The pin to read current from
    int m_pin = -1;

    /// @brief The base voltage of the sensor
    const float m_base_voltage = 2.475f;

    /// @brief The sensitivity of the sensor, taken from the datasheet
    const float m_sensitivity = 0.066f;

    /// @brief The current read from the sensor
    float current = 0;
    /// @brief The old current read from the sensor
    float old_current = 0;
};

extern ACS712 current_sensor;

#endif // ACS712_CURRENT_SENSOR_HPP