#include "utils/safety_state.hpp"
#ifdef _WIN32
#include <cstring>

static size_t strlcpy(char *dst, const char *src, size_t dsize) {
    size_t l = std::strlen(src);
    if (dsize > 0) {
        size_t copylen = (l >= dsize) ? dsize - 1 : l;
        std::memcpy(dst, src, copylen);
        dst[copylen] = '\0';
    }
    return l;
}

static size_t strlcat(char *dst, const char *src, size_t dsize) {
    size_t dlen = std::strlen(dst);
    if (dlen >= dsize) dlen = dsize;
    size_t slen = std::strlen(src);
    size_t rest = dsize - dlen;
    if (rest == 0) return dlen + slen;
    size_t copylen = (slen >= rest) ? rest - 1 : slen;
    std::memcpy(dst + dlen, src, copylen);
    dst[dlen + copylen] = '\0';
    return dlen + slen;
}
#endif

SafetyState safety_state;

uint8_t SafetyState::evaluate(bool transmitter_safety_engaged, bool is_configured, bool is_slow_loop, bool gimbal_power_active) {
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

void SafetyState::reasons_to_string(uint8_t reasons, char* buf, size_t len) {
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
