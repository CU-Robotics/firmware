#include "ACS712.hpp"

float ACS712::read() {
    // equation from datasheet
    // 2.475 is the baseline read with little to no power draw on chassis
    // 3.3 is the voltage of the system
    // 1023 is the max value of the analog read
    float result = (m_base_voltage - ((analogRead(m_pin) * (3.3 / 1023.0)))) / m_sensitivity;
    current = result;
    current = 0.6 * old_current + 0.4 * current;
    
    old_current = current;

    return result;
}