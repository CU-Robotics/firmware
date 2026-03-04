#pragma once
#include <cassert>

namespace safety{
    
    using SafetyFunction = std::function<void()>;

    inline SafetyFunction safety_function_handle() {
        static SafetyFunction safety_function = nullptr;
        return safety_function;
    }

    inline void register_safety_function(SafetyFunction func) {
        safety_function_handle() = std::move(func);
    }

    inline void require(bool condition, const char* message) {
        if (!condition) {
            if (safety_function_handle) {
                safety_function_handle();
            } else{
                Serial.printf("Assert triggered but no safety function registered! Message: %s\n", message);
            }
            assert(condition && message);
        }
    }
}
