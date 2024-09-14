#ifndef SD_WRAP
#define SD_WRAP

#include "../../libraries/SD/SD.h"
#include "../../libraries/SPI/SPI.h"

#define _DIR_LENGTH 128

class SD {
    public:

    SD();

    int open(char* filepath);
    void close();

    int mkfile(char* filename);
    int mkdir(char* dirname);
    int rm(char* filename, bool r);
    int rm(char* filename);

    int read(uint8_t* dest, unsigned int len);
    int write(uint8_t* src, unsigned int len);
    
    bool exists(char* filepath);
    void enumerate_files(char* root, bool r);

    private:

    void enumerate_files(char* root, bool r, int tabs);

    SDClass SDinternal;
    
    File file;
};

#endif