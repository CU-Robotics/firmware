#pragma once
#include <Arduino.h>

enum class LogLevel { INFO, WARN, ERROR };
/// @brief List of robot subsystems
enum class Subsystem { ALL, GENERAL, CAN, MOTORS, SENSORS, ESTIMATOR, COMMS , REF};

struct LogEvent {
    float timestamp;
    LogLevel level;
    Subsystem sys;
    char text[80]; // Max length of message
};

/// @brief Serial wrapper for handling print statements
class SystemLogger : public Print {
private:
    /// @brief number of messages in dashboard box
    static const int LOG_HISTORY = 10;
	/// @brief max length of message in dashbarod box
    static const int MAX_LINE_LEN = 80;
	/// @brief max length including the timestamp
    static const int MAX_STORED_LEN = 100;
	/// @brief message circuluar buffer
    //char messages[LOG_HISTORY][MAX_LINE_LEN] = {{0}};
    LogEvent messages[LOG_HISTORY];
    /// @brief start of the message
    uint8_t head = 0;
	/// @brief number of characters in current message
    uint8_t count = 0;

    /// @brief A temporary buffer to hold the line while print() is building it
    char current_line[MAX_LINE_LEN] = {0};
	/// @brief length of print line in buffer
    uint8_t line_length = 0;
  
    // Context for standard Print() calls
    LogLevel current_level = LogLevel::INFO;
    Subsystem current_sys = Subsystem::GENERAL;
    /// @brief handles whether we print to dashboard or direct to serial
    void push_message();

public:
    /// @brief flag for live printing from CLI
    bool is_live_view_active = false;
    // Dashboard Filters (Defaults to showing everything)
    Subsystem view_filter_sys = Subsystem::ALL;
    LogLevel view_filter_level = LogLevel::INFO;
  

    /// @brief implements print class print for general print statements.
	/// @param c is message to be written
	/// @return the message
    size_t write(uint8_t c) override;
    
	/// @brief implements print class print for println,printf,etc...
	/// @param buffer with message
	/// @param size of message
	/// @return the message
    size_t write(const uint8_t *buffer, size_t size) override;
  
    // --- NEW: Context setter for multiline Print() blocks ---
    void set_context(LogLevel lvl, Subsystem sys);

    // --- NEW: Fast, formatted logger helpers ---
    void info(Subsystem sys, const char* format, ...);
    void warn(Subsystem sys, const char* format, ...);
    void error(Subsystem sys, const char* format, ...);

  
	/// @brief draws dashboard for live prints from CLI
    void draw_dashboard_box();
};

// Declare a global instance so you can use it everywhere, just like 'Serial'
extern SystemLogger SystemLog;
