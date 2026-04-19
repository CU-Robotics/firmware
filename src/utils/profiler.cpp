#include "profiler.hpp"
#include <cstring>

#ifdef PROFILER
static DMAMEM Profiler::profiler_section_t sections[PROF_MAX_SECTIONS] = {0};
#endif

void Profiler::clear() {
#ifdef PROFILER
    for (auto &sec : sections)
        sec = profiler_section_t();
#endif
}
void Profiler::begin(const char *name) {
#ifdef PROFILER
    for (uint32_t i = 0; i < PROF_MAX_SECTIONS; i++) {
        if (strcmp(sections[i].name, name) == 0 || (sections[i].count == 0 && sections[i].name[0] == '\0')) {
            strncpy(sections[i].name, name, PROF_MAX_NAME);
            sections[i].name[PROF_MAX_NAME] = '\0'; 
            sections[i].start_time = micros();
            sections[i].started = 1; 
            return;
        }
    }
#endif
}

void Profiler::end(const char *name) {
#ifdef PROFILER
    for (uint32_t i = 0; i < PROF_MAX_SECTIONS; i++) {
        if (strcmp(sections[i].name, name) == 0) {
            if (!sections[i].started) return;

            uint32_t delta = micros() - sections[i].start_time;

            // Running Average Math
            if (sections[i].count == 0) {
                // First run: set absolute baselines
                sections[i].avg_time = (float)delta;
                sections[i].max_time = delta;
            } else {
                // Subsequent runs: Moving Average
                sections[i].avg_time = (sections[i].avg_time * 0.99f) + ((float)delta * 0.01f);
                
                // Track absolute max
                if (delta > sections[i].max_time) {
                    sections[i].max_time = delta;
                }
            }

            sections[i].count++;
            sections[i].started = 0; 
            return;
        }
    }
#endif
}

void Profiler::print_summary() {
#ifdef PROFILER
    Serial.println("\n================ PROFILER SUMMARY ================");
    Serial.println(" Subsystem        | Avg Time (us) | Max Time (us) ");
    Serial.println("--------------------------------------------------");

    for (uint32_t i = 0; i < PROF_MAX_SECTIONS; i++) {
        if (sections[i].name[0] != '\0' && sections[i].count > 0) {
            
            Serial.printf(" %-16s | %13u | %13u \n", 
                          sections[i].name, 
                          (uint32_t)sections[i].avg_time, 
                          sections[i].max_time);
        }
    }
    Serial.println("==================================================\n");
#endif
}
