#pragma once
#include <cassert>
#include <functional>
#include <Arduino.h>
#include <usb_serial.h>
#include "utils/timing.hpp"

namespace safety {
    
    using SafetyFunction = std::function<void()>;

    inline SafetyFunction& safety_function_handle() {
        static SafetyFunction safety_function = nullptr;
        return safety_function;
    }

    inline void register_safety_function(SafetyFunction func) {
        safety_function_handle() = std::move(func);
    }

    template<typename... Args>
    [[noreturn]] inline void safety_procedure(const char* message, Args... args) {
        Serial.printf("Safety procedure triggered!\n");
        SafetyFunction func = safety_function_handle();
        if (func) {
            func();
        } else{
            Serial.printf("Safety procedure triggered but no safety function registered!\n", message);
        }
        // char buffer[256];
        // Serial.printf(buffer, sizeof(buffer), message, args...);
        Timer timer;
        Serial.printf(message, args...);
        Serial.println();
        while(true){
            // Serial.printf("safety procedure infinite loop\n");
            // timer.delay_millis(1000);
        }
    }

    template<typename... Args>
    inline void assert_or_safety_procedure(bool condition, const char* message, Args... args) {
        if (!condition) {
            safety_procedure(message, args...);
        }
    }
}