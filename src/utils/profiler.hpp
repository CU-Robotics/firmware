#ifndef PROFILER_H
#define PROFILER_H

// IMPORTANT! Use this flag to toggle profiling globally.
// The compiler should optimize out the functions with empty bodies.
// #define PROFILE

// TODO: make 2:1 nested sections work and finisht the PROFILE flag
// TODO: profile main

#include <Arduino.h>

#define PROF_MAX_SECTIONS 4 // max nested profiling sections
#define PROF_MAX_NAME 8 // max length of section name
#define PROF_MAX_TIMES 256 // max number of start/end times per profiler

/// @brief Object for profiling sections of code.
struct Profiler {
    struct profiler_section_t {
        uint32_t start_times[PROF_MAX_TIMES];
        uint32_t end_times[PROF_MAX_TIMES];
        uint8_t count = 0;
        char name[PROF_MAX_NAME + 1]; // extra for null terminator
    };

    profiler_section_t stack[PROF_MAX_SECTIONS];
    int8_t top = -1; // top index, -1 means empty

    /// @brief Start (push) a profiling section onto the stack.
    /// @param name A unique name to identify the section.
    void begin(const char* name) {
    #ifdef PROFILE
        if (top >= PROF_MAX_SECTIONS - 1) return;
        top++;
        strncpy(stack[top].name, name, PROF_MAX_NAME);
        stack[top].name[PROF_MAX_NAME] = '\0'; // ensure null termination
        if (stack[top].count < PROF_MAX_TIMES) {
            stack[top].start_times[stack[top].count] = micros();
        }
    #endif
    }

    /// @brief End (pop) the newest profiling section. Don't call without a corresponding push.
    void end() {
    #ifdef PROFILE
        if (top < 0) return;
        uint8_t count = stack[top].count;
        if (count < PROF_MAX_TIMES) {
            stack[top].end_times[count] = micros();
            stack[top].count++;
        }
        top--;
    #endif
    }

    /// @brief Print stats for a particular profiling section.
    /// @param name The name of the section to print statistics for.
    void print(const char* name) {
    #ifdef PROFILE
        uint32_t min = UINT32_MAX;
        uint32_t max = 0;
        uint32_t sum = 0;

        for (uint8_t i = 0; i < PROF_MAX_SECTIONS; i++) {
            if (strncmp(stack[i].name, name, PROF_MAX_NAME) != 0) continue;
            for (uint8_t j = 0; j < stack[i].count; j++) {
                uint32_t delta = stack[i].end_times[j] - stack[i].start_times[j];
                sum += delta;
                if (delta < min) min = delta;
                if (delta > max) max = delta;
            }
            // Print stats
            if (stack[i].count > 0) {
                Serial.printf("Profiling for: %s\n  Min: %d us\n  Max: %d us\n  Avg: %d us\n",
                                name, min, max, sum / stack[i].count);
            }
            return;
        }
    #endif
    }
};

extern Profiler prof; // Global profiler

#endif // PROFILER_H
