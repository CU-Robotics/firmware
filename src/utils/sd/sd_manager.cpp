#include "sd_manager.hpp"
#include <SdFat.h>


bool SdManager::start() {
    return _sdfat.begin(_SD_CONFIG);
}

void SdManager::stop() {
    _sdfat.end();
}

SdFile SdManager::open_file(const char* path, oflag_t oflag) {
    return SdFile(path, oflag);
}

bool SdManager::file_exists(const char* path) {
    return _sdfat.exists(path);
}
