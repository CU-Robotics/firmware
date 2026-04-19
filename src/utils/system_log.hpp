#pragma once
#include <Arduino.h>

class SystemLogger : public Print {
private:
    static const int LOG_HISTORY = 6;
	static const int MAX_LINE_LEN = 80;    // Max length of the user's print statement
    static const int MAX_STORED_LEN = 100; // Max length including the timestamp
    char messages[LOG_HISTORY][MAX_LINE_LEN] = {0};
    uint8_t head = 0;
    uint8_t count = 0;

    // A temporary buffer to hold the line while print() is building it
    char current_line[MAX_LINE_LEN] = {0};
    uint8_t line_length = 0;

    // Helper to finalize the line and push it to history
    void push_message();

public:
    bool is_live_view_active = false;

    // This is the ONLY function we have to implement to satisfy the Print class
    size_t write(uint8_t c) override;
    
    // Optional but speeds up printf() and String printing significantly
    size_t write(const uint8_t *buffer, size_t size) override; 

    void draw_dashboard_box();
};

// Declare a global instance so you can use it everywhere, just like 'Serial'
extern SystemLogger SystemLog;
