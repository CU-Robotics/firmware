#include <stdio.h>          // printf
#include <fcntl.h>          // open
#include <errno.h>          // errno
#include <stdlib.h>         // malloc
#include <unistd.h>         // close/read/write
#include <sys/time.h>       // gettimeofday
#include <string.h>         // memset
#include <time.h>           // time, tm, mktime, localtime
#include <stdarg.h>         // va_list
#include <sys/stat.h>       // mkdir

#include <fts.h>            // FTS stuff

/// @brief Max path size 
#define PATH_MAX 4096

/// @brief Size of the serial buffers
const size_t read_size = 8ul * 1024ul;

/// @brief Max size of the serial path, - 1 for null terminator
const size_t SERIAL_PATH_SIZE = PATH_MAX - 1;

/// @brief In serial buffer
char* in_buffer = NULL;
/// @brief Out serial buffer
/// @todo Implement writing sometime, this is much more complicated that it seems
char* out_buffer = NULL;

/// @brief Base directory for serial devices on Linux
/// @note No clue why FTS needs it in this format but oh well
char* serial_directory[ ] = { "/dev/serial/by-id/" };

/// @brief Max number of log files to keep
const size_t MAX_LOG_FILES = 5;

/// @brief Prefix for the log file name
const char* LOG_NAME_PREFIX = "teensy_output-";

/// @brief Extension for the log file name
const char* LOG_NAME_EXTENSION = ".log";

/// @brief Base directory for log files
char* log_directory[ ] = { "tools/teensy_logs/" };

/// @brief Find the file path for the Teensy's Serial connection
/// @param serial_path The path to be filled in if found
/// @return -1 on error, 0 on success
int find_teensy_serial_dev(char* serial_path);

/// @brief Create and open a new log file with the correct name. If there are too many log files, replace the oldest one
/// @return The file pointer to the log file to be written to or NULL on error
FILE* open_correct_log_file();

/// @brief Prune the oldest log file
/// @param log_file_names The list of log file path names
void prune_oldest_log_file(char log_file_names[MAX_LOG_FILES][PATH_MAX]);

/// @brief Construct the log file name based on the current time
/// @param log_name The buffer to store the log file name
/// @note The log name format is "teensy_output-MM-DD-HH:MM:SS.log"
void construct_log_name(char log_name[PATH_MAX]);

/// @brief Wrapper for printing to both a file and stdout
/// @param log_file The file to print to
/// @param format The format string to print
/// @param ... The arguments to print
void print(FILE* log_file, const char* format, ...);

// takes no arguments
int main(int argc __attribute__((unused)), char** argv) {
    int exit_code = EXIT_SUCCESS;
    
    // declare the serial device path
    char teensy_dev_path[SERIAL_PATH_SIZE];
    memset(teensy_dev_path, 0, SERIAL_PATH_SIZE);

    // find the teensy's device path
    printf("Looking for Teensy's serial device...\n");

    // loop to find the serial device. time is used to print a message every second
    // we dont use sleep because we want to immediately connect as soon as the device is found
    time_t start_time = time(NULL);

    while (find_teensy_serial_dev(teensy_dev_path) == -1) {
        if (time(NULL) - start_time >= 1) {
            printf("Could not find Teensy's serial path. Retrying...\n");
            start_time = time(NULL);
        }
    }

    printf("Found Teensy's serial device: %s\n", teensy_dev_path);

    // try to open the file
    int dev_flags = O_RDWR | O_NOCTTY | O_TRUNC;

    // try to open the serial connection
    int dev_serial = open(teensy_dev_path, dev_flags);
    if (dev_serial == -1) {
        printf("%s: Failed to open file: %s\n", argv[0], teensy_dev_path);
        
        exit_code = EXIT_FAILURE;
        goto cleanup;
    }

    // allocate the in_buffer
    in_buffer = malloc(read_size);
    if (in_buffer == NULL) { 
        printf("%s: Failed to allocate in_buffer\n", argv[0]);

        exit_code = EXIT_FAILURE;
        goto cleanup; 
    }

    // allocate the out_buffer
    out_buffer = malloc(read_size);
    if (out_buffer == NULL) { 
        printf("%s: Failed to allocate out_buffer\n", argv[0]);

        exit_code = EXIT_FAILURE;
        goto cleanup; 
    }

    // create output file
    FILE* log_file = open_correct_log_file();
    if (log_file == NULL) {
        printf("%s: Failed to open log file\n", argv[0]);

        exit_code = EXIT_FAILURE;
        goto cleanup;
    }

    printf("Monitoring Teensy's serial device...\n");

    // main loop for reading and printing
    while (1) {
        // read as much as we can into in_buffer
        // normally the data is at most chunks of 4KB
        int read_ret = read(dev_serial, in_buffer, read_size);

        // if read_ret is positive, it means we read
        // if it is 0, the connection has been lost
        // if it is -1, some unrecoverable error occured
        if (read_ret > 0) {
            in_buffer[read_ret] = 0;

            // print the buffer, do not append a newline
            print(log_file, "%s", in_buffer);
        } else if (read_ret == 0) {
            // read returning 0 indicates the serial connection is over (its EOF but on a FIFO stream)
            // we need to re-try the connection
            print(log_file, "Teensy's serial disconnecting. Attempting to reconnect...\n");
            close(dev_serial);
            dev_serial = -1;

            // infintely try to regain connection. User can kill the program if it doesnt work
            do {
                // try to find the device, it wont exist if teensy is down
                if (find_teensy_serial_dev(teensy_dev_path) == -1) {
                    continue;
                }

                print(log_file, "Attempting to open...");
                dev_serial = open(teensy_dev_path, dev_flags);

                // print the status on open
                if (dev_serial == -1) print(log_file, "failed\n");
                else print(log_file, "Succeeded\n");
            }
            while (dev_serial == -1);

        } else {
            print(log_file, "Error in monitoring: ");
            perror("Read failed:");

            exit_code = EXIT_FAILURE;
            goto cleanup;
        }
    }

    // lil bit of goto buisness
    // cleanup the buffers and close the serial connection
cleanup:
    // clean buffer memory
    free(in_buffer);
    free(out_buffer);

    // close the serial connection
    close(dev_serial);

    return exit_code;
}

int find_teensy_serial_dev(char* serial_path) {
    FTS* traversal_pointer = NULL;
    FTSENT* current_file = NULL;
    FTSENT* child_files = NULL;

    // no clue what these do, but FTS_NOCHDIR makes it not recursive
    int fts_options = FTS_COMFOLLOW | FTS_LOGICAL | FTS_NOCHDIR;

    // open the base directory
    if ((traversal_pointer = fts_open(serial_directory, fts_options, NULL)) == NULL) {
        printf("Failed to open path: %s\n", *serial_directory);
        fts_close(traversal_pointer);
        return -1;
    }

    // fill ftsp's internal structure with all of the items in the directory
    child_files = fts_children(traversal_pointer, 0);
    if (child_files == NULL) {
        printf("No files to traverse in %s directory\n", *serial_directory);
        fts_close(traversal_pointer);
        return -1;
    }

    // whether we've found the serial device in the base path
    int serial_found = 0;

    // iterate through all of the "things" in this directory
    while ((current_file = fts_read(traversal_pointer)) != NULL) {
        // check whether this file is likely a symbol (not a file nor a directory)
        if (current_file->fts_info == FTS_DEFAULT) {
            // look for the correct interface number "-if00" for Serial (not SerialUSB1)
            char* substr_pos = strstr(current_file->fts_path, "-if00");
            if (substr_pos == NULL) {
                continue;
            }

            char path_name[SERIAL_PATH_SIZE];

            // convert the symbol path to an actual path
            int size = readlink(current_file->fts_path, path_name, SERIAL_PATH_SIZE);
            if (size == -1) {
                printf("Readlink failed to find correct path\n");
                break;
            }

            // append null terminator after path
            path_name[size] = 0;

            serial_found = 1;

            // put the device file path into path_name
            strncpy(serial_path, path_name, SERIAL_PATH_SIZE);

            break;
        }
    }

    fts_close(traversal_pointer);

    // could not find a path to the correct serial
    if (!serial_found) {
        // printf("Could not find Teensy's serial path\n");
        return -1;
    }

    // the path that readlink gives us is relative to /dev/serial/by-id (it gives us something like ../../blah)
    // we need to append /dev/serial/by-id/ to the path for it to be functional
    char full_path[SERIAL_PATH_SIZE];
    memset(full_path, 0, SERIAL_PATH_SIZE);

    // sanity check that the sizes of both strings will not overflow
    size_t rel_path_size = strlen(serial_path);
    size_t base_path_size = strlen("/dev/serial/by-id/");
    if (rel_path_size + base_path_size > SERIAL_PATH_SIZE) {
        printf("Path size too large\n");
        fts_close(traversal_pointer);
        exit(-1);
    }

    // construct the full path from the base and the symlink's relative path
    strcpy(full_path, "/dev/serial/by-id/");
    strcat(full_path, serial_path);

    // this produces a path like: /dev/serial/by-id/../../blah
    strncpy(serial_path, full_path, SERIAL_PATH_SIZE);

    return 0;
}

FILE* open_correct_log_file() {
    char log_file_names[MAX_LOG_FILES][PATH_MAX];
    memset(log_file_names, 0, sizeof(log_file_names));

    // number of log files found
    size_t log_count = 0;

    // FTS traversal and file pointers
    FTS* traversal_pointer = NULL;
    FTSENT* current_file = NULL;
    FTSENT* child_files = NULL;

    // no clue what these do, but FTS_NOCHDIR makes it not recursive
    int fts_options = FTS_COMFOLLOW | FTS_LOGICAL | FTS_NOCHDIR;

    // open the base directory
    if ((traversal_pointer = fts_open(log_directory, fts_options, NULL)) == NULL) {
        printf("Failed to open path: %s\n", *log_directory);
    }

    // fill ftsp's internal structure with all of the items in the directory
    child_files = fts_children(traversal_pointer, 0);
    if (child_files == NULL) {
        printf("No files to traverse in %s directory\n", *log_directory);
    }

    // iterate through all of the "things" in this directory
    while ((current_file = fts_read(traversal_pointer)) != NULL) {
        // check if this is a file
        if (current_file->fts_info == FTS_F) {
            // see if this file has the log file prefix
            if (strstr(current_file->fts_path, LOG_NAME_PREFIX) == NULL)
                continue;
            
            // add this name to the list of log files
            strncpy(log_file_names[log_count++], current_file->fts_path, PATH_MAX);

            // if we have enough log files, break
            if (log_count >= MAX_LOG_FILES) {
                break;
            }
        }
    }

    // if we have too many log files, prune the oldest one
    if (log_count >= MAX_LOG_FILES) {
        prune_oldest_log_file(log_file_names);
    }

    // construct the new log file name
    char log_name[PATH_MAX];
    construct_log_name(log_name);

    // verify that the log directory exists
    if (mkdir(*log_directory, 0777) == -1) {
        if (errno != EEXIST) {
            printf("Failed to create log directory: %s\n", *log_directory);
        }
    }
    
    // open the log file
    FILE* log_file = fopen(log_name, "w");
    if (log_file == NULL) {
        printf("Failed to open log file: %s\n", log_name);
    }
    
    return log_file;
}

void prune_oldest_log_file(char log_file_names[MAX_LOG_FILES][PATH_MAX]) {
    time_t oldest_time = __LONG_MAX__;
    size_t oldest_index = 0;
    
    // iterate over the possible log files
    for (size_t i = 0; i < MAX_LOG_FILES; i++) {
        // int month, day, hour, min, sec;
        char* log_name = log_file_names[i] + strlen(LOG_NAME_PREFIX) + strlen(*log_directory);

        // create a tm struct to convert the log name to a time_t
        struct tm tm;
        memset(&tm, 0, sizeof(tm));
        
        // parse month
        if (sscanf(log_name, "%d", &tm.tm_mon) != 1) continue;
        // parse day (3 characters after month)
        if (sscanf(log_name += 3, "%d", &tm.tm_mday) != 1) continue;
        // parse hour (3 chars after day)
        if (sscanf(log_name += 3, "%d", &tm.tm_hour) != 1) continue;
        // parse min (3 chars after hour)
        if (sscanf(log_name += 3, "%d", &tm.tm_min) != 1) continue;
        // parse sec (3 chars after min)
        if (sscanf(log_name += 3, "%d", &tm.tm_sec) != 1) continue;

        // convert the tm struct to a time_t
        tm.tm_year = 2025 - 1900;   // the year doesn't actually matter, but having it be 0 makes the time_t negative
        time_t tm_converted = mktime(&tm);

        // compare the time_t to the 
        if (tm_converted < oldest_time) {
            oldest_time = tm_converted;
            oldest_index = i;
        }
    }

    // remove the oldest file
    if (remove(log_file_names[oldest_index]) == -1) {
        printf("Failed to remove oldest log file: %s\n", log_file_names[oldest_index]);
    }
}

void construct_log_name(char log_name[PATH_MAX]) {
    // clear input log name
    memset(log_name, 0, PATH_MAX);

    // set the path to the log directory
    strcat(log_name, *log_directory);

    // set the prefix name
    strcat(log_name, LOG_NAME_PREFIX);
    
    // get the readable time using tm
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);

    // construct the date and time to place into the log
    const int DATE_TIME_LEN = 64;
    char log_date_time[DATE_TIME_LEN];
    memset(log_date_time, 0, DATE_TIME_LEN);
    // format is MM-DD-HH:MM:SS
    snprintf(log_date_time, DATE_TIME_LEN, "%.2d-%.2d-%.2d:%.2d:%.2d", tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

    // set the date/time
    strcat(log_name, log_date_time);

    // set the extension
    strcat(log_name, LOG_NAME_EXTENSION);
}

void print(FILE* log_file, const char* format, ...) {
    // print to log file
    va_list args;
    va_start(args, format);

    vfprintf(log_file, format, args);

    va_end(args);

    // print to stdout
    va_list args2;
    va_start(args2, format);

    vprintf(format, args2);

    va_end(args2);
}
