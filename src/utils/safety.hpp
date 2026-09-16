#pragma once
#include <Arduino.h>
#include <utility>
#include <functional>

/// @brief How many times per second the LED should blink when a safety procedure is triggered
constexpr int SAFETY_PROCEDURE_LED_BLINK_FREQ = 5; 
/// @brief How long the LED should stay on/off when blinking during a safety procedure (in milliseconds)
constexpr int SAFETY_PROCEDURE_LED_BLINK_DURATION_MS = 1000 / (2 * SAFETY_PROCEDURE_LED_BLINK_FREQ);

namespace safety {
    /// @brief Bitmask of reasons the robot is in safety mode. NONE means the motors are armed.
    namespace Reason {
        enum : uint8_t {
            NONE = 0,
            TRANSMITTER = 1 << 0,           ///< Transmitter safety switch is engaged
            NOT_CONFIGURED = 1 << 1,        ///< Hive has not configured the robot yet
            SLOW_LOOP = 1 << 2,             ///< The last main loop overran its time budget
            GIMBAL_POWER_OFF = 1 << 3,      ///< Ref system reports gimbal power is off
            GIMBAL_POWER_SETTLING = 1 << 4, ///< Gimbal power turned on too recently for the motors to be ready
        };
    }

    /// @brief Buffer size that fits every reason name from reasons_to_string
    constexpr size_t REASON_STR_LEN = 96;

    /// @brief Write a space-separated list of the reason names in a bitmask, or "none"
    /// @param reasons safety::Reason bitmask
    /// @param buf Output buffer, always null-terminated
    /// @param len Size of buf, REASON_STR_LEN fits every reason
    inline void reasons_to_string(uint8_t reasons, char* buf, size_t len) {
        static const struct { uint8_t bit; const char* name; } names[] = {
            {Reason::TRANSMITTER, "transmitter"},
            {Reason::NOT_CONFIGURED, "not-configured"},
            {Reason::SLOW_LOOP, "slow-loop"},
            {Reason::GIMBAL_POWER_OFF, "gimbal-power-off"},
            {Reason::GIMBAL_POWER_SETTLING, "gimbal-power-settling"},
        };
        if (len == 0) return;
        buf[0] = '\0';
        if (reasons == Reason::NONE) {
            strlcpy(buf, "none", len);
            return;
        }
        for (const auto& n : names) {
            if (!(reasons & n.bit)) continue;
            if (buf[0] != '\0') strlcat(buf, " ", len);
            strlcat(buf, n.name, len);
        }
    }

    /// @brief Type definition for our safety function    
    using SafetyFunction = std::function<void()>;

    /// @brief Get a reference to the static safety function handle
    /// @return A reference to the static safety function handle
    inline SafetyFunction& safety_function_handle() {
        static SafetyFunction safety_function = nullptr;
        return safety_function;
    }

    /// @brief Get a reference to the static safety mode active flag
    /// @return A reference to the static boolean indicating if safety mode is currently active
    inline bool& is_safety_mode_active() {
        static bool safety_mode_active = false;
        return safety_mode_active;
    }

    /// @brief Get a reference to the static safety reason bitmask
    /// @return A reference to the safety::Reason bitmask from the most recent safety check
    inline uint8_t& active_reasons() {
        static uint8_t reasons = Reason::NONE;
        return reasons;
    }

    /// @brief Record why the robot is in safety mode; safety mode is active whenever any reason is set
    /// @param reasons safety::Reason bitmask, Reason::NONE to arm the motors
    inline void set_safety_reasons(uint8_t reasons) {
        active_reasons() = reasons;
        is_safety_mode_active() = (reasons != Reason::NONE);
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

    /// @brief Trigger the safety procedure, which will call the registered safety function and then enter an infinite loop. If a safety function is not registered, it will immediately enter the infinite loop.
    /// @param message The message to print when the safety procedure is triggered, which can include format specifiers for the additional arguments
    /// @param args Variadic arguments to be formatted into the message
    /// @tparam Args The types of the variadic arguments
    template<typename... Args>
    [[noreturn]] inline void safety_procedure(const char* message, Args&&... args) {
        Serial.printf("Safety procedure triggered!\n");
        if (!call_safety_function()) {
            Serial.printf("Safety procedure triggered but no safety function registered!\n");
        }

        Serial.printf(message, args...);
        Serial.println();
        while(true) {
            // Blink the teensy's LED to indicate a safety procedure has been triggered
            digitalWrite(LED_BUILTIN, HIGH);
            delay(SAFETY_PROCEDURE_LED_BLINK_DURATION_MS);
            digitalWrite(LED_BUILTIN, LOW);
            delay(SAFETY_PROCEDURE_LED_BLINK_DURATION_MS);
        }
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
