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
    void begin(const char* name) {
    #ifdef PROFILE
        // search for a section by name or an empty slot
        for (uint8_t i = 0; i < PROF_MAX_SECTIONS; i++) {
            if (strncmp(sections[i].name, name, PROF_MAX_NAME) == 0 || sections[i].count == 0) {
                // start this section
                strncpy(sections[i].name, name, PROF_MAX_NAME);
                sections[i].name[PROF_MAX_NAME] = '\0';  // ensure null termination
                if (sections[i].count < PROF_MAX_TIMES) {
                    sections[i].start_times[sections[i].count] = micros();
                }
                return;
            }
        }
    #endif
    }

    /// @brief End a profiling section.
    /// @param name The name of the section to end.
    void end(const char* name) {
    #ifdef PROFILE
        // find the section by name and add an end time
        for (uint8_t i = 0; i < PROF_MAX_SECTIONS; i++) {
            if (strncmp(sections[i].name, name, PROF_MAX_NAME) == 0) {
                uint8_t count = sections[i].count;
                if (count < PROF_MAX_TIMES) {
                    sections[i].end_times[count] = micros();
                    sections[i].count++;
                }
                return;
            }
        }
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
            if (strncmp(sections[i].name, name, PROF_MAX_NAME) != 0) continue;
            for (uint8_t j = 0; j < sections[i].count; j++) {
                uint32_t delta = sections[i].end_times[j] - sections[i].start_times[j];
                sum += delta;
                if (delta < min) min = delta;
                if (delta > max) max = delta;
            }
            // print stats
            if (sections[i].count > 0) {
                Serial.printf("Profiling for: %s\n  Min: %d us\n  Max: %d us\n  Avg: %d us\n",
                                name, min, max, sum / sections[i].count);
            }
            return;
        }
    #endif
    }
};

extern Profiler prof;  // Global profiler

#endif  // PROFILER_H
