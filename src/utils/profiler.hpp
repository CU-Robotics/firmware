#ifndef PROFILER_H
#define PROFILER_H

// Use this flag to toggle profiling globally.
// #define PROFILE

#include <Arduino.h>

#define PROF_MAX_SECTIONS 8  // max number of active profiling sections
#define PROF_MAX_NAME 16      // max length of section name
#define PROF_MAX_TIMES (5000)   // max number of start/end times per section

/// @brief Object for profiling sections of code.
struct Profiler {
};

extern Profiler prof;  // Global profiler

#endif  // PROFILER_H
