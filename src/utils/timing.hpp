#ifndef TIMING_H
#define TIMING_H

#include <Arduino.h>

// Time conversion helpers
#define NS_TO_US(ns)    (ns / 1e3f)
#define NS_TO_MS(ns)    (ns / 1e6f)
#define NS_TO_S(ns)     (ns / 1e9f)

#define US_TO_NS(us)    (us * 1e3f)
#define US_TO_MS(us)    (us / 1e3f)
#define US_TO_S(us)     (us / 1e6f)

#define MS_TO_S(ms)     (ms / 1e3f)
#define MS_TO_US(ms)    (ms * 1e3f)
#define MS_TO_NS(ms)    (ms * 1e6f)

#define S_TO_MS(s)      (s * 1e3f)
#define S_TO_US(s)      (s * 1e6f)
#define S_TO_NS(s)      (s * 1e9f)

/// @brief Timer class. Operates based on micros() and provides a simple interface for timing operations
struct Timer {
    /// @brief Last recorded time, used in duration calculations
    uint32_t start_time = micros();

    /// @brief Default constructor, starts the timer
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
        float delta = US_TO_S(get_elapsed());    // time in s since last start() call
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
        float delta = US_TO_MS(get_elapsed());  // time in ms since last start() call
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