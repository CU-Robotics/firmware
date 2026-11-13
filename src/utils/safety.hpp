#pragma once
#include <utility>
#include <concepts>
#include <functional>
#include <type_traits>
#include <cstdint>

#ifdef FW_SIMULATION
#include <cstdio>
#include <stdexcept>
#else
#include <Arduino.h>
#endif

/// @brief How many times per second the LED should blink when a safety procedure is triggered
constexpr int SAFETY_PROCEDURE_LED_BLINK_FREQ = 5; 
/// @brief How long the LED should stay on/off when blinking during a safety procedure (in milliseconds)
constexpr int SAFETY_PROCEDURE_LED_BLINK_DURATION_MS = 1000 / (2 * SAFETY_PROCEDURE_LED_BLINK_FREQ);

namespace safety {
    /// @brief Severity of a safety message and the corresponding firmware response.
    /// @note Release firmware treats every severity as Warning.
    enum class errorSeverity : std::uint8_t {
        Info,    ///< Print an informational message and continue.
        Warning, ///< Print a warning message and continue.
        Error,   ///< Invoke the safety callback, print an error, and halt normal execution.
        Critical ///< Invoke the safety callback, print a critical error, and halt normal execution.
    };

    /// @brief Compile-time filter for arguments accepted by the safety message functions.
    /// @details Accepts integral and floating-point types regardless of cv/ref qualifiers,
    /// and types convertible to const char*.
    /// @tparam T The argument type to check.
    /// @note Checks argument types only; it does not validate format strings.
    template <typename T>
    concept printable = std::integral<std::remove_cvref_t<T>> || std::floating_point<std::remove_cvref_t<T>> || std::convertible_to<T, const char *>;

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

    /// @brief Print the severity and message; Error and Critical also trigger the safety callback and halt. Simulation always throws.
    /// @note Release firmware prints every severity as Warning and returns without invoking the safety callback.
    /// @param message The message to print when the safety procedure is triggered, which can include format specifiers for the additional arguments
    /// @param errortype The severity, which determines whether to return or halt
    /// @param args Variadic arguments to be formatted into the message
    /// @tparam Args The types of the variadic arguments
    template<typename... Args>
        requires(printable<Args> && ...)
    inline void safety_procedure(const char* message, errorSeverity errortype, Args &&... args) {
    #ifdef FW_SIMULATION

        // throw in simulation for graceful error handling
        char message_buffer[512];
        std::snprintf(message_buffer, sizeof(message_buffer), message, std::forward<Args>(args)...);
        throw std::runtime_error(message_buffer);

    #else

    #ifdef FW_RELEASE
        errortype = errorSeverity::Warning;
    #endif

        const char* severity_name = "Unknown";
        switch (errortype) {
            case errorSeverity::Info: severity_name = "Info"; break;
            case errorSeverity::Warning: severity_name = "Warning"; break;
            case errorSeverity::Error: severity_name = "Error"; break;
            case errorSeverity::Critical: severity_name = "Critical"; break;
        }
        const bool should_halt = errortype != errorSeverity::Info && errortype != errorSeverity::Warning;

        if (should_halt && !call_safety_function()) {
            Serial.printf("Safety procedure triggered but no safety function registered!\n");
        }

        Serial.printf("%s: ", severity_name);
        Serial.printf(message, args...);
        Serial.println();

        if (should_halt) {
            while (true) {
                // Blink the teensy's LED to indicate a safety procedure has been triggered
                digitalWrite(LED_BUILTIN, HIGH);
                delay(SAFETY_PROCEDURE_LED_BLINK_DURATION_MS);
                digitalWrite(LED_BUILTIN, LOW);
                delay(SAFETY_PROCEDURE_LED_BLINK_DURATION_MS);
            }
        }

    #endif
    }

    /// @brief Assert a condition or trigger the safety procedure if the condition is not met
    /// @param condition The condition to assert
    /// @param errortype The severity, which determines whether to return or halt
    /// @param message The message to print if the condition is not met
    /// @param args Variadic arguments to be formatted into the message
    /// @tparam Args The types of the variadic arguments
    template<typename... Args>
        requires(printable<Args> && ...)
    inline void assert_or_safety_procedure(bool condition, errorSeverity errortype, const char* message, Args&&... args) {
        if (!condition) {
            safety_procedure(message, errortype, std::forward<Args>(args)...);
        }
    }
}
