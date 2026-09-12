#pragma once

#include <SdFat.h>

/// @brief Manages FAT filesystem reads/writes
class SdManager {
public:
    /// @brief Initialize SD card reader from chip select pin
    /// @param config Configuration 
    SdManager(const SdioConfig& config) : _SD_CONFIG(config) {};

    /// @brief Stops the SD card manager
    ~SdManager() { stop(); }

    /// @brief Starts the SD card manager
    /// @return `true` if SD card initialized properly, else `false`
    bool start();

    /// @brief Stops the SD card manager
    void stop();

    /// @brief Check if a file exists on the SD card
    /// @param path Path to file
    /// @return `true` if exists, else `false`
    bool file_exists(const char* path);    

    /// @brief Opens a file on the SD card
    /// @param path Path to the file
    /// @param oflag File open flags (8-bit bitfield)
    /// @return Requested file. Existence check with `file.exists()`
    SdFile open_file(const char* path, oflag_t oflag);

private:
    const SdioConfig _SD_CONFIG;    

    SdFat _sdfat;
};

/// @brief `SdManager` instance for the builtin SD card reader
extern SdManager BuiltinSd;
