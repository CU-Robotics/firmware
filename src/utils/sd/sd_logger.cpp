#include "sd_logger.hpp"
#include "utils/system_log.hpp"
#include <cstring>

#define LOG_FILE_DIR "/logs/"
#define LOG_FILE_FORMAT "log_%d.log"

SdLogger BuiltinSdLogger(BuiltinSd);

const char* level_to_str(LogLevel level) {
    switch(level) {
        case LogLevel::WARN: return "WARN";
        case LogLevel::ERROR: return "ERROR";
        default: return "INFO";
    };
}

bool SdLogger::start() {
    return new_log_file();
}

bool SdLogger::write_log(LogEvent& event) {
    if (!_log_file) return false;
    
    // build string
    char log_buffer[128];
    snprintf(log_buffer, sizeof(log_buffer), 
        "%f : %s : %s : %s",
        event.timestamp,
        level_to_str(event.level),
        sys_to_str(event.sys),
        event.text
    );

    // write to file
    if (_log_file.write(log_buffer) < 0)
        return false;

    // sync write to SD card
    return _log_file.sync();
}

bool SdLogger::new_log_file() {
    SdFile log_dir = _sd_man.open_file(LOG_FILE_DIR, O_RDONLY);
    if (!log_dir) return false;

    SdFile curr_file;
    
    // get next log file number
    int num = 1;
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
