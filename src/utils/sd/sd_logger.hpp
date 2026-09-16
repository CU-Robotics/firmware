#pragma once

#include "utils/sd/sd_manager.hpp"
#include "utils/system_log.hpp"

class SdLogger {
public:
    /// @brief Creates a new SdLogger, and a new file on the SD card with it
    /// @param sd_man `SdManager` to use
    SdLogger(SdManager& sd_man) : _sd_man(sd_man) { new_log_file(); }

    ~SdLogger() { _log_file.close(); }

    /// @brief Write a `LogEvent` to the SD card
    /// @param event `LogEvent` to log
    /// @return If write was successful
    bool write_log(LogEvent& event);

private:
    SdManager& _sd_man;

    // File to write to, opened with O_APPEND
    SdFile _log_file;

    // Opens a new log file
    // Format specified in sd_loggr.cpp
    bool new_log_file();
};

extern SdLogger BuiltinSdLogger;
