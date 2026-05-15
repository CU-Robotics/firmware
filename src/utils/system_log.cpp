#include "system_log.hpp"

// Instantiate the global logger
SystemLogger SystemLog;

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
    // This override makes bulk printing (like Strings) much faster
    for (size_t i = 0; i < size; i++) {
        write(buffer[i]);
    }
    return size;
}

void SystemLogger::push_message() {
    current_line[line_length] = '\0'; // Null-terminate the string

    // Prepend the timestamp and format it into the actual circular buffer
	snprintf(messages[head], MAX_STORED_LEN, "[%7.2fs] %s", millis() / 1000.0f, current_line);

    // If the dashboard is CLOSED, print immediately to the scrolling terminal
    if (!is_live_view_active) {
        Serial.println(messages[head]);
    }

    // Advance the circular buffer indices
    head = (head + 1) % LOG_HISTORY;
    if (count < LOG_HISTORY) count++;

    // Reset the temporary line buffer for the next message
    line_length = 0; 
}

void SystemLogger::draw_dashboard_box() {
    Serial.println("============= SYSTEM EVENT LOG =============");
    if (count == 0) {
        Serial.println(" No recent events.");
    } else {
        // Print from oldest to newest
        uint8_t start = (count == LOG_HISTORY) ? head : 0;
        for (uint8_t i = 0; i < count; i++) {
            uint8_t idx = (start + i) % LOG_HISTORY;
            Serial.printf(" %s\n", messages[idx]);
        }
    }
    Serial.println("============================================");
}
