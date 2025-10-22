#ifndef MAGNET_HPP
#define MAGNET_HPP
#include <Arduino.h>

/// @brief definition of magnet class. Controls state of magnet (on/off)

/// Magnet power pin on spring spring board
constexpr int MAGNET_PIN = 5;

class Magnet {
    private:
        // True = On, False = Off
        bool state;
    public:
        Magnet();

        /// @brief Turns the dart launcher's magnet on.
        void on();

        /// @brief Turns the dart launcher's magnet off.
        void off();

        /// @brief Gets the current state of the magnet
        bool get_state();
};

#endif