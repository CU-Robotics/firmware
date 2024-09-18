#ifndef SD_WRAP
#define SD_WRAP

#include "../../libraries/SD/SD.h"
#include "../../libraries/SPI/SPI.h"

#define _DIR_LENGTH 128

class SDManager {
    public:

    SDManager();

    int open(const char* filepath);
    void close();

    int mkfile(const char* filename);
    int mkdir(const char* dirname);
    int rm(const char* filename, bool r);
    int rm(const char* filename);

    int read(uint8_t* dest, unsigned int len);
    int write(uint8_t* src, unsigned int len);
    
    bool exists(const char* filepath);
    void enumerate_files(const char* root, bool r);

    private:

    void enumerate_files(const char* root, bool r, int tabs);

    SDClass SDinternal;
    
    File file;
};

#endif