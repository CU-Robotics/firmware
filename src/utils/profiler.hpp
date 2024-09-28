#ifndef PROFILER_H
#define PROFILER_H

// Use this flag to toggle profiling globally.
#define PROFILE

#include <Arduino.h>

#define PROF_MAX_SECTIONS 8  // max number of active profiling sections
#define PROF_MAX_NAME 16      // max length of section name
#define PROF_MAX_TIMES (5000)   // max number of start/end times per section

/// @brief Object for profiling sections of code.
struct Profiler {
    /// @brief Data structure for a profiling section. 
    struct profiler_section_t {
        /// @brief Start time for each profiling section.
        uint32_t start_times[PROF_MAX_TIMES] = { 0 };
        /// @brief End time for each profiling section.
        uint32_t end_times[PROF_MAX_TIMES] = { 0 };
        /// @brief Number of start/end times recorded (how many deltas can be calculated).
        uint16_t count = 0;
        /// @brief Flag on whether count has overflowed or not
        uint8_t overflow : 7;
        /// @brief Flag on whether this section had begin() called on it
        uint8_t started : 1;
        /// @brief A unique name to identify the section.
        char name[PROF_MAX_NAME + 1] = { 0 };  // extra for null terminator
    };

    /// @brief Start a profiling section.
    /// @param name A unique name to identify the section.
    void begin(const char* name);

    /// @brief End a profiling section.
    /// @param name The name of the section to end.
    void end(const char* name);

    /// @brief Print stats for a particular profiling section.
    /// @param name The name of the section to print statistics for.
    void print(const char* name);
};

extern Profiler prof;  // Global profiler

#endif  // PROFILER_H
