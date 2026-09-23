#include "sd_manager.hpp"

#ifndef BUILTIN_SDCARD_CONFIG
#define BUILTIN_SDCARD_CONFIG SdioConfig(FIFO_SDIO)
#endif // BUILTIN_SDCARD


// define global SD manager for builtin SD card
SdManager BuiltinSd(BUILTIN_SDCARD_CONFIG);

bool SdManager::start() {
    return _sdfat.begin(_SD_CONFIG);
}

void SdManager::stop() {
    _sdfat.end();
}

SdFile SdManager::open_file(const char* path, oflag_t oflag) {
    return SdFile(path, oflag);
}
