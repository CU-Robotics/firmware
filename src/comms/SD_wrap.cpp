#include "SD_wrap.hpp"

#define FILE_NOT_FOUND 1
#define RM_ERR_DIR_MISC 2
#define RM_ERR_FILE_MISC 3

SD::SD(){
    if(!SDinternal.begin()){
        Serial.println("SD_ERROR::SD card initialization failed");
        // TODO: perhaps there should be a better error handler here?? 
    };
}

int SD::open(char* filepath){
    // check if exists
    if(!exists(filepath)) {
        Serial.print("SD_NOTICE::filepath ");
        Serial.print(filepath);
        Serial.println(" does not exist or contains no file");
        return 1;
    }
    file = SDinternal.open(filepath);
    return 0;
}

void SD::close(){
    file.close();
    return;
}

int SD::mkfile(char* filename){
    if(SDinternal.exists(filename)){
        Serial.print("SD_NOTICE::file located at ");
        Serial.print(filename);
        Serial.println(" already exists");
        return 1;
    }
    if(!SDinternal.open(filename)){
        Serial.print("SD_NOTICE::file located at ");
        Serial.print(filename);
        Serial.println(" could not be created (directory may not exist)");
        return 1;
    };
    return 0;
}

int SD::mkdir(char* dirname){
    if(!SDinternal.mkdir(dirname)){
        Serial.print("SD_NOTICE::directory ");
        Serial.print(dirname);
        Serial.println(" could not be created");
        return 1;
    }
    return 0;
}

int SD::rm(char* filename){
    rm(filename, 0);
}

int SD::rm(char* filename, bool r){
    // check if file or directory
    File cur;
    if(SDinternal.exists(filename)) cur = SDinternal.open(filename);
    else{
        Serial.print("SD_NOTICE::file at ");
        Serial.print(filename);
        Serial.println(" could not be erased (no such file or directory)");
        return FILE_NOT_FOUND;
    }

    if(cur.isDirectory()){
        // attempt to remove dir
        if(!r){     // non-recursive erase
            if(!SDinternal.rmdir(filename)){
                Serial.print("SD_NOTICE::file at ");
                Serial.print(filename); 
                Serial.println(" could not be erased");
                return RM_ERR_DIR_MISC;
            }
        }
        else{
            // recursive erase
            File tmp = cur.openNextFile;
            while(tmp){
                rm(tmp.name(), 1);
                tmp = cur.openNextFile();
            }
        }
    }
    else{
        // attempt to remove file
        if(!SDinternal.remove(filename)){
            Serial.print("SD_NOTICE::file at ");
            Serial.print(filename);
            Serial.println(" could not be erased");
            return RM_ERR_FILE_MISC;
        }
    }
    return 0;
}

int SD::read(uint8_t* dest, unsigned int len){
    if(!file){
        Serial.println("SD_NOTICE::file cannot be read (no file open)");
        return 1;
    }
    file.read(dest, len);
    return 0;
}

int SD::write(uint8_t* src, unsigned int len){
    if(!file){
        Serial.println("SD_NOTICE::file cannot be written to (no file open)");
        return 1;
    }
    file.write(src, len);
    return 0;
}

bool SD::exists(char* filepath)
{
    return SDinternal.exists(filepath);
}

void SD::enumerate_files(char* root, bool r){
    enumerate_files(root, r, 0);
}

void SD::enumerate_files(char* root, bool r, int tabs)
{
    if(!exists(root)){  // dir does not exist
        Serial.print("SD_NOTICE::directory ");
        Serial.print(root);
        Serial.println(" cannot be enumerated (directory does not exist)");
        return;
    }

    File dir = SDinternal.open(root);
    if(!dir.isDirectory()){     // dir is not a directory
        Serial.print("SD_NOTICE::data at ");
        Serial.print(root);
        Serial.println(" is a file");
        return;
    }

    for(int i = 0; i < tabs; i++){
        Serial.print("\t");
    }
    Serial.println(root);

    File tmp = dir.openNextFile();
    while(tmp){
        for(int i = 0; i < tabs + 1; i++){
            Serial.print("\t");
        }
        Serial.println(tmp.name());

        if(tmp.isDirectory()){
            enumerate_files(tmp.name(), 1, tabs + 1);
        }

        tmp = dir.openNextFile();
    }
}
