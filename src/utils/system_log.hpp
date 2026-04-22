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

    /// @brief handles whether we print to dashboard or direct to serial
    void push_message();

public:
    bool is_live_view_active = false;

    /// @brief implements print class print for general print statements.
    size_t write(uint8_t c) override;
    
	/// @brief implements print class print for println,printf,etc...
    size_t write(const uint8_t *buffer, size_t size) override; 
	/// @brief draws dashboard for live prints from CLI
    void draw_dashboard_box();
};

// Declare a global instance so you can use it everywhere, just like 'Serial'
extern SystemLogger SystemLog;
