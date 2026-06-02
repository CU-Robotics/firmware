#include "system_log.hpp"

// Instantiate the global logger
SystemLogger SystemLog;

const char* sys_to_str(Subsystem sys) {
    switch(sys) {
    case Subsystem::CAN: return "CAN";
    case Subsystem::MOTORS: return "MOT";
    case Subsystem::SENSORS: return "SEN";
    case Subsystem::ESTIMATOR: return "EST";
	case Subsystem::COMMS: return "COM";
	case Subsystem::REF: return "REF";
    default: return "GEN";
    }
}

const char* level_to_color(LogLevel lvl) {
    switch(lvl) {
        case LogLevel::ERROR: return "\033[31m"; // Red
        case LogLevel::WARN:  return "\033[33m"; // Yellow
        default:              return "\033[0m";  // Reset/Terminal Default
    }
}

void SystemLogger::set_context(LogLevel lvl, Subsystem sys) {
    current_level = lvl;
    current_sys = sys;
}

size_t SystemLogger::write(uint8_t c) {
    // Ignore carriage returns to prevent weird spacing
    if (c == '\r') return 1; 

    // If we receive a newline (e.g. from println), or if the buffer is full,
    // finalize the message and push it to the circular array.
	if (c == '\n' || line_length >= MAX_LINE_LEN - 1) {
        push_message();
    } else {
        current_line[line_length++] = (char)c;
    }
    
    return 1; // Tell the Print class we successfully wrote 1 byte
}

size_t SystemLogger::write(const uint8_t *buffer, size_t size) {
    for (size_t i = 0; i < size; i++) { write(buffer[i]); }
    return size;
}

void SystemLogger::push_message() {
    current_line[line_length] = '\0'; 

    // 1. Save metadata into the Struct
    messages[head].timestamp = millis() / 1000.0f;
    messages[head].level = current_level;
    messages[head].sys = current_sys;
    strncpy(messages[head].text, current_line, MAX_LINE_LEN);

    // 2. If CLI is closed, print immediately with colors!
    if (!is_live_view_active && should_show(messages[head].level, messages[head].sys)) {
        Serial.printf("[%7.2fs] %s[%s] %s\033[0m\n", 
            messages[head].timestamp, 
            level_to_color(messages[head].level),
            sys_to_str(messages[head].sys), 
            messages[head].text);
    }

    // 3. Advance circular buffer
    head = (head + 1) % LOG_HISTORY;
    if (count < LOG_HISTORY) count++;

    line_length = 0; 
    current_level = LogLevel::INFO;    // Reset to default
    current_sys = Subsystem::GENERAL;  // Reset to default
}
void SystemLogger::log_format(LogLevel lvl, Subsystem sys, const char* format, va_list args) {
    char temp[MAX_LINE_LEN];
    vsnprintf(temp, MAX_LINE_LEN, format, args);
    
    // Strip trailing newlines so it formats perfectly
    size_t len = strlen(temp);
    while(len > 0 && (temp[len-1] == '\n' || temp[len-1] == '\r')) temp[--len] = '\0';

    set_context(lvl, sys);
    strncpy(current_line, temp, MAX_LINE_LEN);
    line_length = len;
    push_message();
}

// --- Info Overloads ---
void SystemLogger::info(Subsystem sys, const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_format(LogLevel::INFO, sys, format, args);
    va_end(args);
}
void SystemLogger::info(const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_format(LogLevel::INFO, Subsystem::GENERAL, format, args);
    va_end(args);
}

// --- Warn Overloads ---
void SystemLogger::warn(Subsystem sys, const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_format(LogLevel::WARN, sys, format, args);
    va_end(args);
}
void SystemLogger::warn(const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_format(LogLevel::WARN, Subsystem::GENERAL, format, args);
    va_end(args);
}

// --- Error Overloads ---
void SystemLogger::error(Subsystem sys, const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_format(LogLevel::ERROR, sys, format, args);
    va_end(args);
}
void SystemLogger::error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_format(LogLevel::ERROR, Subsystem::GENERAL, format, args);
    va_end(args);
}

bool SystemLogger::should_show(LogLevel lvl, Subsystem sys) {
    // Only evaluate if the message meets the minimum requested priority level
    if (lvl >= view_filter_level) {
        // Rule 1: High priority (Warnings & Errors) ALWAYS pierce the subsystem filter
        if (lvl >= LogLevel::WARN) {
            return true; 
        } 
        // Rule 2: Standard INFO messages only show if they match the active subsystem
        else if (view_filter_sys == Subsystem::ALL || sys == view_filter_sys) {
            return true;
        }
    }
    return false;
}

void SystemLogger::draw_dashboard_box() {
    Serial.println("============= SYSTEM EVENT LOG =============");
    if (count == 0) {
        Serial.println(" No recent events.");
    } else {
        uint8_t start = (count == LOG_HISTORY) ? head : 0;
        for (uint8_t i = 0; i < count; i++) {
            uint8_t idx = (start + i) % LOG_HISTORY;
            LogEvent& ev = messages[idx];

            if (should_show(ev.level, ev.sys)) {
                Serial.printf(" [%7.2fs] %s[%s] %s\033[0m\n", 
                    ev.timestamp, 
                    level_to_color(ev.level),
                    sys_to_str(ev.sys), 
                    ev.text);
            }
        }
    }
    Serial.println("============================================");
}
