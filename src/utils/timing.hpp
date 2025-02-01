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

// Timer
// + start()
// + delay_millis(uint32_t duration)
// + delay_micros(uint32_t duration)
// + delta()
// + delta_millis()
// + delta_micros()

// /// @brief Timing object with blocking capability
// struct Timer {
//     /// @brief start time
//     uint32_t t = ARM_DWT_CYCCNT;

//     /// @brief Start time
//     void start_timer() { t = ARM_DWT_CYCCNT; }

//     /// @brief Helper to pause for a duration. Duration starts
//     /// @param duration microseconds to wait (from when any other timing function was called)
//     void delay_micros(uint32_t duration) {
//         while (DURATION_US(t, ARM_DWT_CYCCNT) < duration) {}
//         start_timer();
//     }

//     /// @brief Helper to pause for a duration. Duration starts when startTimer() is called.
//     /// @param duration milliseconds to wait (from when any other timing function was called)
//     void delay_millis(uint32_t duration) {
//         while (DURATION_MS(t, ARM_DWT_CYCCNT) < duration) {}
//         start_timer();
//     }

//     /// @brief delta of clock
//     /// @return deltaTime: (float) The time since the last delta call.
//     float delta() {
//         float delta = DURATION_US(t, ARM_DWT_CYCCNT) / (float)(1E6);
//         start_timer();
//         return delta;
//     }

//     /// @brief delta of clock in microseconds
//     /// @return deltaTime: (float) The time since the last delta call.
//     float delta_micros() {
//         float delta = DURATION_US(t, ARM_DWT_CYCCNT);
//         start_timer();
//         return delta;
//     }
// };



struct Timer {
    /// @brief Last recorded time, used in duration calculations
    uint32_t start_time = micros();

    Timer() { start(); }

    /// @brief Start the timer from the current time
    /// @note This resets all further methods to be relative to the current time
    void start() { start_time = micros(); }

    /// @brief Busy delay for a set duration in us
    /// @param duration The duration in us
    void delay_micros(uint32_t duration) {
        while (!duration_over(duration));   // wait until duration
        start();    // restart the timer for next run
    }

    /// @brief Busy delay for a set duration in ms
    /// @param duration The duration in ms
    void delay_millis(uint32_t duration) {
        while (!duration_over(MS_TO_US(duration))); // wait until duration
        start();    // restart the timer for next run
    }

    /// @brief Get the time in s since last timer call
    /// @note Other calls like delay will cause this function to return the delta since that call
    /// @return The delta in s since last timer method call
    float delta() {
        float delta = get_elapsed() / 100000.f;    // time in s since last start() call
        start();    // restart the timer for next run
        return delta;
    }

    /// @brief Get the time in us since last timer call
    /// @note Other calls like delay will cause this function to return the delta since that call
    /// @return The delta in us since last timer method call
    float delta_micros() {
        float delta = get_elapsed();
        start();    // restart the timer for next run
        return delta;
    }

    /// @brief Get the time in ms since last timer call
    /// @note Other calls like delay will cause this function to return the delta since that call
    /// @return The delta in ms since last timer method call
    float delta_millis() {
        float delta = get_elapsed() / 1000.f;  // time in ms since last start() call
        start();    // restart the timer for next run
        return delta;
    }
    

private:
    /// @brief Helper to use when waiting until a duration is finished
    /// @param duration The duration in microseconds to check for
    /// @note This handles potential overflows within micros()
    /// @return True if the duration is elapsed, else false
    bool duration_over(uint32_t duration) {
        uint32_t curr_time = micros();

        // if curr_time is less than start_time, then this overflowed
        if (curr_time < start_time) {
            // calculate remaining duration
            // elapsed = (max - t1 + 1) + t2 assuming t2 < t1
            // + 1 is from the time going from MAX -> 0, this is still a microsecond
            uint32_t elapsed = (__UINT32_MAX__ - start_time + 1) + curr_time;
            if (elapsed >= duration) return true;
        }

        // curr_time is normal, check like usual
        if (curr_time - start_time >= duration) return true;

        // not elapsed
        return false;
    }

    /// @brief Get delta in us since last call to start()
    /// @note This handles potential overflows within micros()
    /// @return Delta in us
    uint32_t get_elapsed() {
        uint32_t curr_time = micros();
        
        // if curr_time is less than start_time, then this overflowed
        if (curr_time < start_time) {
            // calculate elapsed micros
            // elapsed = (max - t1 + 1) + t2 assuming t2 < t1
            // + 1 is from the time going from MAX -> 0, this is still a microsecond
            return (__UINT32_MAX__ - start_time + 1) + curr_time;

        }

        // return the last time since start() call
        return curr_time - start_time;
    }

};

#endif // TIMING_H