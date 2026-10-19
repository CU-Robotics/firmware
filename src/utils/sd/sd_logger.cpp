#include "sd_logger.hpp"
#include "utils/system_log.hpp"
#include <SdFat.h>
#include <common/FsApiConstants.h>
#include <cstring>

#define LOG_FILE_DIR "/logs/"
#define LOG_FILE_FORMAT "log_%d.log"


bool SdLogger::start() {
    return new_log_file();
}

bool SdLogger::write_log(LogEvent& event) {
    if (!_log_file) return false;
    
    // build string
    char log_buffer[128];
    snprintf(log_buffer, sizeof(log_buffer), 
        "%f : %s : %s : %s\n",
        event.timestamp,
        level_to_str(event.level),
        sys_to_str(event.sys),
        event.text
    );

    // write to file
    if (_log_file.write(log_buffer) <= 0)
        return false;

    // sync write to SD card
    return _log_file.sync();
}

bool SdLogger::new_log_file() {
    if (!_sd_man) return false;

    // create log directory if not exists
    if (!_sd_man->file_exists(LOG_FILE_DIR)) {
        _sd_man->mkdir(LOG_FILE_DIR, "0755");
    }

    SdFile log_dir = _sd_man->open_file(LOG_FILE_DIR, O_RDONLY);
    if (!log_dir) return false;

    SdFile curr_file;
    
    // get next log file number
    int num = 0;
    while (curr_file.openNext(&log_dir, O_RDONLY)) {
        char name_buf[32];
        int num_match;
        curr_file.getName(name_buf, sizeof(name_buf));

        if (!sscanf(name_buf, LOG_FILE_FORMAT, &num_match)) continue;
        num = max(num, num_match);
        
        curr_file.close();
    }
    
    // sub number into format
    char log_file_path[32];
    snprintf(log_file_path, sizeof(log_file_path), LOG_FILE_DIR LOG_FILE_FORMAT, num+1);

    // open the file
    return _log_file.open(log_file_path, O_WRITE | O_APPEND | O_CREAT);
}
