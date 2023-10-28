#include <Arduino.h>

#ifndef TIMING_H
#define TIMING_H

// Some generic converters
#define MS_TO_US(ms) 	(ms * 1000)
#define MS_TO_NS(ms) 	(ms * 1000000)
#define US_TO_NS(us) 	(us * 1000)
#define US_TO_MS(us) 	(us / 1000)
#define NS_TO_US(ns) 	(ns / 1000)
#define NS_TO_MS(ns) 	(ns / 1000000)

// pass ARM_DWT_CYCNT to this to get the timing down to nanoseconds
#define CYCLES_TO_MS(cycles)  ((cycles)*(1E3/F_CPU))
#define CYCLES_TO_US(cycles)  ((cycles)*(1E6/F_CPU))
#define CYCLES_TO_NS(cycles)  ((cycles)*(1E9/F_CPU))

// Get time duration from two cycle counts
#define DURATION_MS(cyccnt1, cyccnt2) (CYCLES_TO_MS(cyccnt2 - cyccnt1))
#define DURATION_US(cyccnt1, cyccnt2) (CYCLES_TO_US(cyccnt2 - cyccnt1))
#define DURATION_NS(cyccnt1, cyccnt2) (CYCLES_TO_NS(cyccnt2 - cyccnt1))

struct Timer {
    uint32_t t = ARM_DWT_CYCCNT;

    void startTimer() { t = ARM_DWT_CYCCNT; }

    void delayMicros(uint32_t duration) {
        /*
        Helper to pause for a duration. Duration starts
        when startTimer() is called.
        @param
        duration: (uint32_t) microseconds to wait (from when startTimer() was called)
        @return
            None
        */
        while(DURATION_US(t, ARM_DWT_CYCCNT) < duration) {}
    }

    void delayMillis(uint32_t duration) {
        /*
        Helper to pause for a duration. Duration starts
        when startTimer() is called.
        @param
        duration: (uint32_t) milliseconds to wait (from when startTimer() was called)
        @return
            None
        */
        while(DURATION_MS(t, ARM_DWT_CYCCNT) < duration) {}
    }
}

#endif