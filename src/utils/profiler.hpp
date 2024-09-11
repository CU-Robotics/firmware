#ifndef PROFILER_H
#define PROFILER_H

// Use this flag to toggle profiling globally.
#define PROFILE

#include <Arduino.h>

#define PROF_MAX_SECTIONS 4  // max number of active profiling sections
#define PROF_MAX_NAME 16      // max length of section name
#define PROF_MAX_TIMES 256   // max number of start/end times per section

/// @brief Object for profiling sections of code.
struct Profiler {
    struct profiler_section_t {
        uint32_t start_times[PROF_MAX_TIMES];
        uint32_t end_times[PROF_MAX_TIMES];
        uint8_t count = 0;
        char name[PROF_MAX_NAME + 1];  // extra for null terminator
    };

    profiler_section_t sections[PROF_MAX_SECTIONS];

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
