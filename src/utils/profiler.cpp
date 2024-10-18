#include "profiler.hpp"

/// @brief Array of profiling sections.
static DMAMEM Profiler::profiler_section_t sections[PROF_MAX_SECTIONS] = { 0 };

void Profiler::begin(const char* name) {
#ifdef PROFILE
    // search for a section by name or an empty slot
    for (uint32_t i = 0; i < PROF_MAX_SECTIONS; i++) {
        if (strncmp(sections[i].name, name, PROF_MAX_NAME) == 0 || (sections[i].count == 0 && sections[i].overflow == 0)) {
            // start this section
            strncpy(sections[i].name, name, PROF_MAX_NAME);
            sections[i].name[PROF_MAX_NAME] = '\0';  // ensure null termination
            if (sections[i].count < PROF_MAX_TIMES) {
                sections[i].start_times[sections[i].count] = micros();
                sections[i].started = 1;    // set this section as "started"
            }
            return;
        }
    }
#endif
}

void Profiler::end(const char* name) {
#ifdef PROFILE
    // find the section by name and add an end time
    for (uint32_t i = 0; i < PROF_MAX_SECTIONS; i++) {
        if (strncmp(sections[i].name, name, PROF_MAX_NAME) == 0) {
            uint32_t count = sections[i].count;
            if (count < PROF_MAX_TIMES) {
                sections[i].end_times[count] = micros();
                sections[i].count++;
                sections[i].started = 0;    // set this section as "finished"
                // if this value just overflowed, or it has reached the max time count, 
                // mark it as overflowed and reset to 0
                if (sections[i].count == 0 || sections[i].count == PROF_MAX_TIMES) {
                    sections[i].overflow = 1;
                    sections[i].count = 0;
                }
            }
            return;
        }
    }
#endif
}

void Profiler::print(const char* name) {
#ifdef PROFILE
    uint32_t min = UINT32_MAX;
    uint32_t max = 0;
    uint32_t sum = 0;

    for (uint32_t i = 0; i < PROF_MAX_SECTIONS; i++) {
        if (strncmp(sections[i].name, name, PROF_MAX_NAME) != 0) continue;
        // find actual count since this value can overflow/max out
        uint32_t actual_count = sections[i].overflow ? PROF_MAX_TIMES : sections[i].count;
        for (unsigned int j = 0; j < actual_count; j++) {
            // if end() was not called when we decide to print, ignore the "bad" section reading
            if (sections[i].started && j == sections[i].count) continue;

            uint32_t delta = sections[i].end_times[j] - sections[i].start_times[j];
            // sum all deltas to be averaged upon printing
            sum += delta;
            if (delta < min) min = delta;
            if (delta > max) max = delta;
        }
        // print stats
        Serial.printf("Profiling for: %s\n  Min: %u us\n  Max: %u us\n  Avg: %u us\n", 
                        name, min, max, sum / actual_count);
        return;
    }
#endif
}
