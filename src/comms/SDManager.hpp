#ifndef SD_WRAP
#define SD_WRAP

#include "../../libraries/SD/SD.h"
#include "../../libraries/SPI/SPI.h"

#define SD_DIR_LENGTH 256

#define FILE_NOT_FOUND 1
#define RM_ERR_DIR_MISC 2
#define RM_ERR_FILE_MISC 3

/// @brief Wrapper for built in SD card
class SDManager {
public:

    /// @brief Constructor, initializes SD objects
    SDManager();

    /// @brief Opens to internal file
    /// @param filepath path to file
    /// @return 0 on success, -1 on failure
    int open(const char* filepath);

    /// @brief Opens to internal file, choose open setting from FILE_READ, FILE_WRITE, FILE_WRITE_APPEND
    /// @param filepath 
    /// @param setting 
    /// @return 0 on success, -1 on failure
    /// @note FILE_READ opens from start of file, FILE_WRITE opens at end and truncates file, FILE_WRITE_APPEND opens at end and appends to file
    int open(const char* filepath, int setting);

    /// @brief Close currently open file
    void close();

    /// @brief Create new file at filename
    /// @param filename path to new file (e.g. /folder/text.txt creates text.txt inside /folder/)
    /// @return 0 on success, -1 on failure
    int touch(const char* filename);

    /// @brief Creates new directory at dirname
    /// @param dirname location of new directory (e.g. /folder/newdir creates directory newdir inside /folder/)
    /// @return 0 on success, -1 on failure
    int mkdir(const char* dirname);

    /// @brief Removes file or directory
    /// @param filename path to file to remove
    /// @param r recursive flag, 1 for recursive removal, 0 for non-recursive 
    /// @return 0 on success, -1 on failure
    /// @note Directory is only removed when empty, or when removed recursively (r==1)
    int rm(const char* filename, bool r);

    /// @brief Removes file or directory
    /// @param filename path to file to remove
    /// @return 0 on success, -1 on failure
    /// @note Overload for removing files by default, sets recursive flag to 0
    int rm(const char* filename);

    /// @brief Reads bytes from file into dest at file location
    /// @param dest destination buffer for data
    /// @param len number of bytes to read
    /// @return number of bytes read
    int read(uint8_t* dest, unsigned int len);

    /// @brief Writes bytes from src into file at file location
    /// @param src source buffer
    /// @param len number of bytes to write
    /// @return number of bytes written
    int write(uint8_t* src, unsigned int len);

    /// @brief Changes location in file based on offset and whence value
    /// @param offset amount to change location by
    /// @param whence setting for lseek configuration (SEEK_SET, SEEK_CUR, SEEK_END)
    /// @return location in file after seek, or -1 on failure
    /// @note SEEK_SET sets location to offset, SEEK_CUR sets location to location + offset, SEEK_END sets location to file size + offset
    int lseek(int offset, int whence);

    /// @brief Checks if file at filepath exists
    /// @param filepath path to file
    /// @return true if exists, false if not
    bool exists(const char* filepath);

    /// @brief Prints all files present in directory root
    /// @param root root directory to enumerate
    /// @param r recursive flag, set true to print contents of all subdirectories
    void enumerate_files(const char* root, bool r);

private:

    /// @brief Prints all files present in directory root
    /// @param root root directory to enumerate
    /// @param r recursive flag, set true to print contents of all subdirectories
    /// @param tabs number of tabs to print (used internally for inner dir printing)
    void enumerate_files(const char* root, bool r, int tabs);

    /// @brief Internal instance of SDClass object used by Teensy
    SDClass SDinternal;

    /// @brief Internal File object that represents currently opened file on SD card
    File file;
};

#endif