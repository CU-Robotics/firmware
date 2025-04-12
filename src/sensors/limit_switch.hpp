#pragma once

#include <Arduino.h>

// tie C pin to ground
// either tie NO pin to ground or nothing
// NC pin is the pin to read and give to this class

/// @brief Class to manage a limit switch
class LimitSwitch {
public:
    /// @brief Constructor for the LimitSwitch class
    /// @param pin The pin number to which the limit switch is connected
    LimitSwitch(int pin) : pin(pin) {
        pinMode(pin, INPUT);
    }

    /// @brief Get the status of the switch
    /// @return true if the switch is pressed, false otherwise
    bool isPressed() {
        return digitalRead(pin);
    }

    /// @brief Set the pin number for the limit switch
    /// @param newPin The new pin number to which the limit switch is connected
    void setPin(int newPin) {
        pin = newPin;
        pinMode(pin, INPUT);
    }
    
private:
    /// @brief The pin number to which the limit switch is connected
    int pin;
};
