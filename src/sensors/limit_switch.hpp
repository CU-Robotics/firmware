#pragma once

#include <Arduino.h>

// tie C pin to 3.3v
// NO pin is the pin to read, it is set up as input pulldown 
// NC should be untied

/// @brief Class to manage a limit switch
class LimitSwitch {
public:
    /// @brief Constructor for the LimitSwitch class
    /// @param pin The pin number to which the limit switch is connected
    LimitSwitch(int pin) : pin(pin) {
        pinMode(pin, INPUT_PULLDOWN);
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
