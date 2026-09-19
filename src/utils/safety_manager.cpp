#include "utils/safety_manager.hpp"

SafetyManager safety_manager;

uint8_t SafetyManager::evaluate(bool transmitter_safety_engaged, bool is_configured, bool is_slow_loop, bool gimbal_power_active) {
    if (gimbal_power_active && !m_last_gimbal_power) {
        m_gimbal_power_timer.start();
    }
    m_last_gimbal_power = gimbal_power_active;

    uint8_t reasons = Reason::NONE;
    if (transmitter_safety_engaged)
        reasons |= Reason::TRANSMITTER;
    if (!is_configured)
        reasons |= Reason::NOT_CONFIGURED;
    if (is_slow_loop)
        reasons |= Reason::SLOW_LOOP;
    if (!gimbal_power_active) {
        reasons |= Reason::GIMBAL_POWER_OFF;
    } else if (m_gimbal_power_timer.get_elapsed_micros_no_restart() < GIMBAL_POWER_SETTLE_US) {
        reasons |= Reason::GIMBAL_POWER_SETTLING;
    }

    m_active_reasons = reasons;
    return reasons;
}

void SafetyManager::reasons_to_string(uint8_t reasons, char* buf, size_t len) {
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
