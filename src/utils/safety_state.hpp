#pragma once
#include <Arduino.h>

#include "utils/timing.hpp"

/// @brief How long to stay disarmed after gimbal power turns on, to let the motors boot.
constexpr uint32_t GIMBAL_POWER_SETTLE_US = 3000000;

/// @brief Owns the robot's safety mode (arming) state: whether the motors may be driven and why not.
/// @note This is distinct from safety.hpp, which handles fatal safety procedures that halt the robot.
class SafetyState {
  public:
    /// @brief Bitmask of reasons the robot is in safety mode. NONE means the motors are armed.
    struct Reason {
        /// @brief The individual reason bits
        enum : uint8_t {
            NONE = 0,
            TRANSMITTER = 1 << 0,           ///< Transmitter safety switch is engaged
            NOT_CONFIGURED = 1 << 1,        ///< Hive has not configured the robot yet
            SLOW_LOOP = 1 << 2,             ///< The last main loop overran its time budget
            GIMBAL_POWER_OFF = 1 << 3,      ///< Ref system reports gimbal power is off
            GIMBAL_POWER_SETTLING = 1 << 4, ///< Gimbal power turned on too recently for the motors to be ready
        };
    };

    /// @brief Buffer size that fits every reason name from reasons_to_string
    static constexpr size_t REASON_STR_LEN = 96;

    /// @brief Evaluate every arming condition and latch the result
    /// @param transmitter_safety_engaged Whether the transmitter's safety switch is engaged
    /// @param is_configured Whether Hive has configured the robot
    /// @param is_slow_loop Whether the last main loop overran its time budget
    /// @param gimbal_power_active Whether the ref system reports gimbal power is on
    /// @return The new Reason bitmask, Reason::NONE if the motors may be armed
    uint8_t evaluate(bool transmitter_safety_engaged, bool is_configured, bool is_slow_loop, bool gimbal_power_active);

    /// @brief Get the reason bitmask from the most recent evaluate call
    /// @return A Reason bitmask, Reason::NONE if the motors are armed
    uint8_t active_reasons() const { return m_active_reasons; }

    /// @brief Whether safety mode is currently active, i.e. any reason is set
    /// @return true if the robot is in safety mode
    bool is_safety_mode_active() const { return m_active_reasons != Reason::NONE; }

    /// @brief Whether the motors are armed and allowed to move
    /// @return true if no reason is set
    bool motors_armed() const { return m_active_reasons == Reason::NONE; }

    /// @brief Write a space-separated list of the reason names in a bitmask, or "none"
    /// @param reasons Reason bitmask
    /// @param buf Output buffer, always null-terminated
    /// @param len Size of buf, REASON_STR_LEN fits every reason
    static void reasons_to_string(uint8_t reasons, char* buf, size_t len);

  private:
    /// @brief The Reason bitmask from the most recent evaluate call
    uint8_t m_active_reasons = Reason::NONE;

    /// @brief Cache of the previous evaluate call's gimbal power state to detect changes
    bool m_last_gimbal_power = false;

    /// @brief Tracks how long gimbal power has been active
    Timer m_gimbal_power_timer;
};

// Declare a global instance so you can use it everywhere, just like 'SystemLog'
extern SafetyState safety_state;
