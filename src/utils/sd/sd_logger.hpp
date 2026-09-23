#pragma once

#include "utils/sd/sd_manager.hpp"
#include "utils/system_log.hpp"

/// @brief Creates and manages a log file on the SD card
class SdLogger {
public:
    /// @brief Creates a new SdLogger, and a new file on the SD card with it
    /// @param sd_man `SdManager` to use
    SdLogger(SdManager& sd_man) : _sd_man(sd_man) {}

    /// @brief Closes the file associated with the logger
    ~SdLogger() { _log_file.close(); }

    /// @brief Creates and binds a file to the logger
    /// @return Whether the start successfully created a new file
    bool start();

    /// @brief Write a `LogEvent` to the SD card
    /// @param event `LogEvent` to log
    /// @return If write was successful
    bool write_log(LogEvent& event);

private:
    /// @brief SD manager used for file management
    SdManager& _sd_man;

    /// @brief File to write to, opened with O_APPEND
    SdFile _log_file;

    /// @brief Opens a new log file
    /// @return Whether file was created and bound to successfully
    bool new_log_file();
};

extern SdLogger BuiltinSdLogger;
