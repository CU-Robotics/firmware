#pragma once
#include <cassert>
#include <functional>

namespace safety {
    
    using SafetyFunction = std::function<void()>;

    inline SafetyFunction& safety_function_handle() {
        static SafetyFunction safety_function = nullptr;
        return safety_function;
    }

    inline void register_safety_function(SafetyFunction func) {
        safety_function_handle() = std::move(func);
    }

    [[noreturn]] inline void safety_procedure(const char* message) {
        SafetyFunction func = safety_function_handle();
        if (func) {
            func();
        } else{
            Serial.printf("Safety procedure triggered but no safety function registered! Message: %s\n", message);
        }
        assert(false && message);
    }

    inline void assert_or_safety_procedure(bool condition, const char* message) {
        if (!condition) {
            safety_procedure(message);
        }
    }
}