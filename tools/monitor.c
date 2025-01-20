#include <stdio.h>      // printf
#include <fcntl.h>      // open
#include <errno.h>      // errno
#include <stdlib.h>     // malloc
#include <unistd.h>     // close/read/write
#include <sys/time.h>   // gettimeofday
#include <string.h>     // memset

#include <fts.h>        // FTS stuff

/// @brief Size of the serial buffers
const long read_size = 8 * 1024;

/// @brief In serial buffer
char* in_buffer = NULL;
/// @brief Out serial buffer
/// @todo Implement writing sometime, this is much more complicated that it seems
char* out_buffer = NULL;

/// @brief base directory for serial devices on linux
/// @note No clue why FTS needs it in this format but oh well
char* serial_directory[ ] = { "/dev/serial/by-id/" };

/// @brief Find the file path for the Teensy's Serial connection
/// @param serial_path The path to be filled in if found
/// @return -1 on error, 0 on success
int find_teensy_serial_dev(char* serial_path);

// takes no arguments
int main(int argc, char** argv) {
    int exit_code = EXIT_SUCCESS;
    
    // declare the serial device path
    char teensy_dev_path[128];
    memset(teensy_dev_path, 0, 128);

    // find the teensy's device path
    if (find_teensy_serial_dev(teensy_dev_path) == -1) {
        printf("%s: Could not find Teensy's serial device\n", argv[0]);
        return EXIT_FAILURE;
    }

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
    if (in_buffer == NULL) { perror("in_malloc_alloc"); goto cleanup; }

    // allocate the out_buffer
    out_buffer = malloc(read_size);
    if (out_buffer == NULL) { perror("out_malloc_alloc"); goto cleanup; }

    // main loop for reading and printing
    while (1) {
        // read as much as we can into in_buffer
        int read_ret = read(dev_serial, in_buffer, read_size);

        // if read_ret is positive, it means we read
        // if it is 0, the connection has been lost
        // if it is -1, some unrecoverable error occured
        if (read_ret > 0) {
            // get current time in milliseconds
            struct timeval tv;
            gettimeofday(&tv, NULL);
            long long time_in_mill = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;

            in_buffer[read_ret] = 0;

            printf("%s", in_buffer);
        } else if (read_ret == 0) {
            // read returning 0 indicates the serial connection is over (its EOF but on a FIFO stream)
            // we need to re-try the connection
            printf("Teensy's serial disconnecting. Attempting to reconnect...\n");
            close(dev_serial);
            dev_serial = -1;

            // infintely try to regain connection. User can kill the program if it doesnt work
            do {
                // try to find the device, it wont exist if teensy is down
                if (find_teensy_serial_dev(teensy_dev_path) == -1) {
                    continue;
                }

                printf("Attempting to open...");
                dev_serial = open(teensy_dev_path, dev_flags);

                // print the status on open
                if (dev_serial == -1) printf("failed\n");
                else printf("Succeeded\n");
            }
            while (dev_serial == -1);

        } else {
            printf("Error in monitoring: ");
            perror("Read failed:");

            exit_code = EXIT_FAILURE;
            goto cleanup;
        }
    }

    // lil bit of goto buisness
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
        return -1;
    }

    // fill ftsp's internal structure with all of the items in the directory
    child_files = fts_children(traversal_pointer, 0);
    if (child_files == NULL) {
        printf("No files to traverse in %s directory\n", *serial_directory);
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

            char path_name[128];

            // convert the symbol path to an actual path
            int size = readlink(current_file->fts_path, path_name, 128);
            if (size == -1) {
                printf("Readlink failed to find correct path\n");
                break;
            }

            // append null terminator after path
            path_name[size] = 0;

            serial_found = 1;

            // put the device file path into path_name
            strncpy(serial_path, path_name, 128);

            break;
        }
    }

    // could not find a path to the correct serial
    if (!serial_found) {
        // printf("Could not find Teensy's serial path\n");
        return -1;
    }

    // the path that readlink gives us is relative to /dev/serial/by-id (it gives us something like ../../blah)
    // we need to append /dev/serial/by-id/ to the path for it to be functional
    char full_path[128];
    memset(full_path, 0, 128);

    // sanity check that the sizes of both strings will not overflow
    int rel_path_size = strlen(serial_path);
    int base_path_size = strlen("/dev/serial/by-id/");
    if (rel_path_size + base_path_size > 128) {
        printf("Path size too large\n");
        exit(-1);
    }

    // construct the full path from the base and the symlink's relative path
    strcpy(full_path, "/dev/serial/by-id/");
    strcat(full_path, serial_path);

    printf("Found Teensy's serial path: %s\n", full_path);

    // this produces a path like: /dev/serial/by-id/../../blah
    strncpy(serial_path, full_path, 128);

    return 0;
}