#pragma once
#include <Arduino.h>

enum class LogLevel { INFO, WARN, ERROR };
/// @brief List of robot subsystems
enum class Subsystem { ALL, GENERAL, CAN, MOTORS, SENSORS, ESTIMATOR, COMMS , REF, Controls};
/// @brief Event log struct which holds all relevent data about potential print
struct LogEvent {
    /// @brief current loop count
    float timestamp;
    /// @brief Priority Level
    LogLevel level;
    /// @brief Message's subsystem
    Subsystem sys;
    /// @brief contains message with max length 80 characters
    char text[80];
};

/// @brief Converts Subsystem enum to cstring
/// @param sys Subsystem to convert
/// @return cstring representaiton of `sys`
const char* sys_to_str(Subsystem sys);

/// @brief Serial wrapper for handling print statements
class SystemLogger : public Print {
private:
    /// @brief number of messages in dashboard box
    static const int LOG_HISTORY = 5;
	/// @brief max length of message in dashboard box
    static const int MAX_LINE_LEN = 80;
	/// @brief max length including the timestamp
    static const int MAX_STORED_LEN = 100;
	/// @brief message circuluar buffer
    LogEvent messages[LOG_HISTORY];
    /// @brief start of the message
    uint8_t head = 0;
	/// @brief number of characters in current message
    uint8_t count = 0;

    /// @brief A temporary buffer to hold the line while print() is building it
    char current_line[MAX_LINE_LEN] = {0};
	/// @brief length of print line in buffer
    uint8_t line_length = 0;
	/// @brief pointer to link CLI to logger so print statements arent interuppted
    char* shared_cli_buffer = nullptr;
  
    /// @brief Context for standard Print() calls
    LogLevel current_level = LogLevel::INFO;
    /// @brief Subsystem for standard print call
    Subsystem current_sys = Subsystem::GENERAL;
    /// @brief handles whether we print to dashboard or direct to serial
    void push_message();
    /// @brief string formatter so we don't repeat code
    /// @param lvl is the urgency level of log
    /// @param sys is the subsystem
    /// @param format is the message
    /// @param args is the variadic formatting options
    void log_format(LogLevel lvl, Subsystem sys, const char *format, va_list args);
    /// @brief set context level and subsystem of interest for print statement
    /// @param lvl is the urgency level of log
    /// @param sys is the subsystem
    void set_context(LogLevel lvl, Subsystem sys);
    /// @brief logic for determing wether a message should print or not
    /// @param lvl is the urgency level of log
    /// @param sys is the subsystem
    /// @return whether we show the message or not
    bool should_show(LogLevel lvl, Subsystem sys);
public:
    /// @brief flag for live printing from CLI
    bool is_live_view_active = false;
    /// @brief Dashboard System Filters (Defaults to showing everything)
    Subsystem view_filter_sys = Subsystem::ALL;
    /// @brief Dashboard Level Filters (Defaults to showing everything)
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
    /// @brief buffer info level message into queue
	/// @param sys is the subsytem that will be coupled with the message
	/// @param format is any printf variadic formatting information
    void info(Subsystem sys, const char *format, ...);
    /// @brief buffer warn level message into queue
	/// @param sys is the subsytem that will be coupled with the message
	/// @param format is any printf variadic formatting information
    void warn(Subsystem sys, const char *format, ...);
    /// @brief buffer error level message into queue
	/// @param sys is the subsytem that will be coupled with the message
	/// @param format is any printf variadic formatting information
    void error(Subsystem sys, const char* format, ...);
    /// @brief buffer info level message into queue
    /// @param format is any printf variadic formatting information
    /// @note defaults to general subsystem
    void info(const char *format, ...);
    /// @brief buffer warn level message into queue
    /// @param format is any printf variadic formatting information
    /// @note defaults to general subsystem
    void warn(const char *format, ...);
    /// @brief buffer error level message into queue
    /// @param format is any printf variadic formatting information
    /// @note defaults to general subsystem
    void error(const char* format, ...);
	/// @brief draws dashboard for live prints from CLI
    void draw_dashboard_box();
    /// @brief Call this once at boot to link the CLI buffer to the logger
    /// @param buffer is a pointer to CLI buffer
    void bind_cli_buffer(char* buffer) { shared_cli_buffer = buffer; }
};

// Declare a global instance so you can use it everywhere, just like 'Serial'
extern SystemLogger SystemLog;
