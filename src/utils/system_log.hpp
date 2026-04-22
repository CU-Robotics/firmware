#pragma once
#include <Arduino.h>
/// @brief Serial wrapper for handling print statements
class SystemLogger : public Print {
private:
	/// @brief number of messages in dashboard box
    static const int LOG_HISTORY = 6;
	/// @brief max length of message in dashbarod box
	static const int MAX_LINE_LEN = 80;
	/// @brief max length including the timestamp
    static const int MAX_STORED_LEN = 100;
	/// @brief message buffer
    char messages[LOG_HISTORY][MAX_LINE_LEN] = {0};
	/// @brief start of the message
    uint8_t head = 0;
	/// @brief number of characters in current message
    uint8_t count = 0;

    /// @brief A temporary buffer to hold the line while print() is building it
    char current_line[MAX_LINE_LEN] = {0};
	/// @brief length of print line in buffer
    uint8_t line_length = 0;

    /// @brief handles whether we print to dashboard or direct to serial
    void push_message();

public:
	/// @brief flag for live printing from CLI
    bool is_live_view_active = false;

    /// @brief implements print class print for general print statements.
	/// @brief c is message to be written
    size_t write(uint8_t c) override;
    
	/// @brief implements print class print for println,printf,etc...
	/// @param buffer with message
	/// @param size of message
    size_t write(const uint8_t *buffer, size_t size) override; 
	/// @brief draws dashboard for live prints from CLI
    void draw_dashboard_box();
};

// Declare a global instance so you can use it everywhere, just like 'Serial'
extern SystemLogger SystemLog;
