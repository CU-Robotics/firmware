#ifndef TIMING_H
#define TIMING_H

#include <Arduino.h>

// Some generic converters
#define MS_TO_US(ms) 	(ms * 1000)
#define MS_TO_NS(ms) 	(ms * 1000000)
#define US_TO_NS(us) 	(us * 1000)
#define US_TO_MS(us) 	(us / 1000)
#define NS_TO_US(ns) 	(ns / 1000)
#define NS_TO_MS(ns) 	(ns / 1000000)

// pass ARM_DWT_CYCNT to this to get the timing down to nanoseconds
#define CYCLES_TO_S(cycles)   (CYCCNT_OVERFLOW(cycles)*(F_CPU))
#define CYCLES_TO_MS(cycles)  (CYCCNT_OVERFLOW(cycles)*(1E3/F_CPU))
#define CYCLES_TO_US(cycles)  (CYCCNT_OVERFLOW(cycles)*(1E6/F_CPU))
#define CYCLES_TO_NS(cycles)  (CYCCNT_OVERFLOW(cycles)*(1E9/F_CPU))

// Get time duration from two cycle counts
#define DURATION_S(cyccnt1, cyccnt2)  (CYCLES_TO_S(cyccnt2 - cyccnt1))
#define DURATION_MS(cyccnt1, cyccnt2) (CYCLES_TO_MS(cyccnt2 - cyccnt1))
#define DURATION_US(cyccnt1, cyccnt2) (CYCLES_TO_US(cyccnt2 - cyccnt1))
#define DURATION_NS(cyccnt1, cyccnt2) (CYCLES_TO_NS(cyccnt2 - cyccnt1))

// Accounts for overflow
#define UINT_MAX 4294967295
#define CYCCNT_OVERFLOW(duration) (duration > UINT_MAX*0.25 ? UINT_MAX-duration : duration)

/// @brief Timing object with blocking capability
struct Timer {
    /// @brief start time
    uint32_t t = ARM_DWT_CYCCNT;

    /// @brief Start time
    void start_timer() { t = ARM_DWT_CYCCNT; }

    /// @brief Helper to pause for a duration. Duration starts
    /// @param duration microseconds to wait (from when any other timing function was called)
    void delay_micros(uint32_t duration) {
        while (DURATION_US(t, ARM_DWT_CYCCNT) < duration) {}
        start_timer();
    }

    /// @brief Helper to pause for a duration. Duration starts when startTimer() is called.
    /// @param duration milliseconds to wait (from when any other timing function was called)
    void delay_millis(uint32_t duration) {
        while (DURATION_MS(t, ARM_DWT_CYCCNT) < duration) {}
        start_timer();
    }

    /// @brief delta of clock
    /// @return deltaTime: (float) The time since the last delta call.
    float delta() {
        float delta = DURATION_US(t, ARM_DWT_CYCCNT) / (float)(1E6);
        start_timer();
        return delta;
    }

    /// @brief delta of clock in microseconds
    /// @return deltaTime: (float) The time since the last delta call.
    float delta_micros() {
        float delta = DURATION_US(t, ARM_DWT_CYCCNT);
        start_timer();
        return delta;
    }
};

#endif // TIMING_H