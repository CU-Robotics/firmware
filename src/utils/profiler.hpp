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
    /// @brief Constructor for the profiler.
    Profiler() {
        clear();
    }

    // @brief Data structure for a profiling section
    struct profiler_section_t {
        /// @brief Start time for each profiling section
	uint32_t start_times[PROF_MAX_TIMES] = {0};
	/// @brief End time for each profiling section
	uint32_t end_times[PROF_MAX_TIMES] = {0};
	/// @brief Number of start/end times recorded
	uint16_t count = 0;
	/// @brief Label on if count has overflowed
	uint8_t overflowed = 0;
	/// @brief Label on if a begin() has been called and the corresponding end() hasn't yet
	uint8_t started = 0;
	/// @brief Name for each section
	char name[PROF_MAX_NAME] = {0};
    };

    /// @brief clear all profiling sections
    void clear();

    /// @brief Start a profiling section
    /// @param name A unique name to identify the section
    void begin(const char* name);

    /// @brief End a profiling section
    /// @param name The name of the section to end
    void end(const char* name);

    /// @brief Print stats for a profiling section
    /// @param name The name of the section to print stats for
    void print(const char* name);
};

extern Profiler prof;  // Global profiler

#endif  // PROFILER_H
