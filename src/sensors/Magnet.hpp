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

        // Turns magnet on
        void on();

        // Turns magnet off
        void off();

        // Returns current state of magnet
        bool get_state();
};

#endif