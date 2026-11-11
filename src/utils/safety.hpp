#pragma once
#include <concepts>
#include <functional>
#include <type_traits>
#include <utility>

#ifdef FW_SIMULATION
#include <cstdio>
#include <stdexcept>
#else
#include <Arduino.h>
#endif

/*
Imp a bool return type so you can choose to terminate the program or not in saftey predure
*/

/// @brief How many times per second the LED should blink when a safety procedure is triggered
constexpr int SAFETY_PROCEDURE_LED_BLINK_FREQ = 5; 
/// @brief How long the LED should stay on/off when blinking during a safety procedure (in milliseconds)
constexpr int SAFETY_PROCEDURE_LED_BLINK_DURATION_MS = 1000 / (2 * SAFETY_PROCEDURE_LED_BLINK_FREQ);

template <typename T>
concept printable = std::integral<std::remove_cvref_t<T>> || std::floating_point<std::remove_cvref_t<T>> || std::convertible_to<T, const char *>;

namespace safety {
    /// @brief Type definition for our safety function    
    using SafetyFunction = std::function<void()>;

    /// @brief Get a reference to the static safety function handle
    /// @return A reference to the static safety function handle
    inline SafetyFunction& safety_function_handle() {
        static SafetyFunction safety_function = nullptr;
        return safety_function;
    }

    /// @brief Register a safety function to be called when a safety procedure is triggered
    /// @param func The safety function to register
    inline void register_safety_function(SafetyFunction func) {
        safety_function_handle() = std::move(func);
    }

    /// @brief Call the registered safety function and return true. If no safety function is registered, return false.
    /// @return true if a safety function was registered and invoked, false otherwise.
    inline bool call_safety_function() {
        SafetyFunction &func = safety_function_handle();
        if (!func) { return false; }

        func();
        return true;
    }

    /// @brief Trigger the safety procedure, which will call the registered safety function and then just prints a message
    /// @param message The message to print when the safety procedure is triggered, which can include format specifiers for the additional arguments
    /// @param args Variadic arguments to be formatted into the message
    /// @tparam Args The types of the variadic arguments
    template <typename... Args>
        requires(printable<Args> && ...)
    inline void safety_procedure(const char* message, Args &&... args) {
    #ifdef FW_SIMULATION

        // throw in simulation for graceful error handling
        char message_buffer[512];
        std::snprintf(message_buffer, sizeof(message_buffer), message, std::forward<Args>(args)...);
        throw std::runtime_error(message_buffer);

    #else

        Serial.printf("Safety procedure triggered!\n");
        if (!call_safety_function()) {
            Serial.printf("Safety procedure triggered but no safety function registered!\n");
        }

    Serial.printf(message, args...);
    Serial.println();

    #endif
    }

    /// @brief Assert a condition or trigger the safety procedure if the condition is not met
    /// @param condition The condition to assert
    /// @param message The message to print if the condition is not met
    /// @param args Variadic arguments to be formatted into the message
    /// @tparam Args The types of the variadic arguments
    template<typename... Args>
    inline void assert_or_safety_procedure(bool condition, const char* message, Args&&... args) {
        if (!condition) {
            safety_procedure(message, std::forward<Args>(args)...);
        }
    }
}
