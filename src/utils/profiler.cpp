#include "profiler.hpp"
#include <cstring>

#ifdef PROFILE
static DMAMEM Profiler::profiler_section_t sections[PROF_MAX_SECTIONS] = {0};
#endif

void Profiler::clear() {
#ifdef PROFILE
    for (auto &sec : sections)
        sec = profiler_section_t();
#endif
}

void Profiler::begin(const char *name) {
#ifdef PROFILE
    //find section by name or first empty slot
    for(uint32_t i = 0; i < PROF_MAX_SECTIONS; i++) {
        if(strcmp(sections[i].name, name) == 0 || (sections[i].count == 0 && sections[i].overflowed == 0)) {
            //start section
            strncpy(sections[i].name, name, PROF_MAX_NAME-1);
            sections[i].start_time = micros();
            sections[i].started = 1;//label section as started
            return;
        }
    }
#endif
}

void Profiler::end(const char *name) {
#ifdef PROFILE
    //find section by name
    for(uint32_t i = 0; i < PROF_MAX_SECTIONS; i++) {
        if(strcmp(sections[i].name, name) == 0) {
            //end section
            if(sections[i].count < PROF_MAX_TIMES) {
                sections[i].time_lengths[sections[i].count] = micros() - sections[i].start_time;
                sections[i].count++;
                sections[i].started = 0;//label section as finished
                
                //if count is now at limit, reset count and label as overflowed
                if(sections[i].count == PROF_MAX_TIMES)
                {
                    sections[i].count = 0;
                    sections[i].overflowed = 1;
                }
            }
            return;
        }
    }
#endif
}

void Profiler::print(const char *name) {
#ifdef PROFILE
    uint32_t sum = 0;
    uint32_t min = UINT32_MAX;
    uint32_t max = 0;
    
    // find section by name
    for (uint32_t i = 0; i < PROF_MAX_SECTIONS; i++) {
        if (strcmp(sections[i].name, name) == 0) {
            // calculate values
            uint32_t trueCount = sections[i].overflowed ? PROF_MAX_SECTIONS : sections[i].count;
            for (uint32_t j = 0; j < trueCount; j++) {
                // if the last run was started and not ended, ignore it
                if (sections[i].started && j == sections[i].count)
                    continue;
                
                uint32_t delta = sections[i].time_lengths[j];
                sum += delta;
                if(delta < min)
                    min = delta;
                if(delta > max)
                    max = delta;
            }
            
            // print values
            Serial.printf("Profiling for: %s\n Min: %u us\n Max: %u us\n Avg: %u us\n", name, min, max,
                          sum / trueCount);
            return;
        }
    }
#endif
}
