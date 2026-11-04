#pragma once

#include <SdCard/SdioCard.h>
#include <SdFat.h>

#ifndef BUILTIN_SDCARD_CONFIG
#define BUILTIN_SDCARD_CONFIG SdioConfig(FIFO_SDIO)
#endif // BUILTIN_SDCARD


/// @brief Manages FAT filesystem reads/writes
class SdManager {
public:
    /// @brief Initializes teensy builtin SD card reader
    SdManager() : _SD_CONFIG(BUILTIN_SDCARD_CONFIG) {}
    
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

    /// @brief Opens a file on the SD card
    /// @param path Path to the file
    /// @param oflag File open flags (8-bit bitfield)
    /// @return Requested file
    SdFile open_file(const char* path, oflag_t oflag);


    /// @brief Checks if a file exists on the SD card
    /// @param path File path to check
    /// @return Whether the file exists
    bool file_exists(const char* path);

    /// @brief Creates a directory at the specified location
    /// @param path Path of the directory to create
    /// @param pflag Pflags for directory creation
    /// @return If directory creation was successful
    bool mkdir(const char* path, const char* pflag);

private:
    /// @brief SdioConfig used to initialize the manager
    const SdioConfig _SD_CONFIG;

    /// @brief SdFat object used in manager
    SdFat _sdfat;
};
