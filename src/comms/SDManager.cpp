#include "SDManager.hpp"

SDManager::SDManager() {
    if (!SDinternal.begin()) {
        Serial.println("SD_ERROR::SDManager card initialization failed");
        // TODO: perhaps there should be a better error handler here?? 
    }
}

int SDManager::open(const char* filepath, int setting) {
    // check if exists
    if (!exists(filepath)) {
        Serial.print("SD_NOTICE::filepath ");
        Serial.print(filepath);
        Serial.println(" does not exist or contains no file");
        return -1;
    }
    if (file) file.close();
    file = SDinternal.open(filepath, setting);
    return 0;
}

int SDManager::open(const char* filepath) {
    return open(filepath, FILE_WRITE_APPEND);
}

void SDManager::close() {
    file.close();
    return;
}

int SDManager::touch(const char* filename) {
    if (SDinternal.exists(filename)) {
        Serial.print("SD_NOTICE::file located at ");
        Serial.print(filename);
        Serial.println(" already exists");
        return -1;
    }
    if (!SDinternal.open(filename, FILE_WRITE)) {
        Serial.print("SD_NOTICE::file located at ");
        Serial.print(filename);
        Serial.println(" could not be created (directory may not exist)");
        return -1;
    };
    return 0;
}

int SDManager::mkdir(const char* dirname) {
    if (!SDinternal.mkdir(dirname)) {
        Serial.print("SD_NOTICE::directory ");
        Serial.print(dirname);
        Serial.println(" could not be created");
        return -1;
    }
    return 0;
}

int SDManager::rm(const char* filename) {
    return rm(filename, 0);
}

int SDManager::rm(const char* filename, bool r) {
    // check if file or directory
    File cur;
    if (SDinternal.exists(filename)) cur = SDinternal.open(filename);
    else {
        Serial.print("SD_NOTICE::file at ");
        Serial.print(filename);
        Serial.println(" could not be erased (no such file or directory)");
        return FILE_NOT_FOUND;
    }

    if (cur.isDirectory()) {
        // attempt to remove dir
        if (!r) {     // non-recursive erase
            if (!SDinternal.rmdir(filename)) {
                Serial.print("SD_NOTICE::file at ");
                Serial.print(filename);
                Serial.println(" could not be erased");
                return RM_ERR_DIR_MISC;
            }
        } else {
            // recursive erase
            File tmp = cur.openNextFile();
            while (tmp) {
                char name[SD_DIR_LENGTH];
                memset(name, 0, SD_DIR_LENGTH);
                memcpy(name, filename, strlen(filename));
                if (!(filename[strlen(filename) - 1] == '/')) strcat(name, "/");
                strcat(name, tmp.name());
                rm(name, 1);
                tmp = cur.openNextFile();
            }
            SDinternal.rmdir(filename);
        }
    } else {
        // attempt to remove file
        if (!SDinternal.remove(filename)) {
            Serial.print("SD_NOTICE::file at ");
            Serial.print(filename);
            Serial.println(" could not be erased");
            return RM_ERR_FILE_MISC;
        }
    }
    return 0;
}

int SDManager::read(uint8_t* dest, unsigned int len) {
    if (!file) {
        Serial.println("SD_NOTICE::file cannot be read (no file open)");
        return -1;
    }
    return file.read(dest, len);
}

int SDManager::write(uint8_t* src, unsigned int len) {
    if (!file) {
        Serial.println("SD_NOTICE::file cannot be written to (no file open)");
        return -1;
    }
    return file.write(src, len);
}

int SDManager::lseek(int offset, int whence) {
    int flag = 0;
    switch (whence) {
    case SEEK_SET:
        flag = file.seek(offset);
        break;
    case SEEK_CUR:
        flag = file.seek(file.position() + offset);
        break;
    case SEEK_END:
        flag = file.seek(file.size() + offset);
        break;
    }
    if (flag) return file.position();
    else return -1;
}

bool SDManager::exists(const char* filepath) {
    // check whether the sd card is actually there, and if it previously wasn't but is now (ie it got inserted after upload), re-initialize the card
    // this is a SD.h internal function and is badly named. oh well
    SDinternal.mediaPresent();
    
    return SDinternal.exists(filepath);
}

void SDManager::enumerate_files(const char* root, bool r) {
    enumerate_files(root, r, 0);
}

void SDManager::enumerate_files(const char* root, bool r, int tabs) {
    // check current

    if (!exists(root)) {
        Serial.print("SD_NOTICE::file ");
        Serial.print(root);
        Serial.println(" does not exist");
        return;
    }

    File dir = SDinternal.open(root);

    // print current

    for (int i = 0; i < tabs; i++) {
        Serial.print("-\t");
    }
    Serial.print(dir.name());

    // if not dir, then exit
    if (!dir.isDirectory()) {
        Serial.println("");
        return;
    }

    Serial.println(" (dir)");   // label directory

    // if not recursive, then exit
    if ((!r) && tabs > 0) return;

    // open tmp file, and loop through all contents of dir
    File tmp = dir.openNextFile();

    while (tmp) {
        // print current file
        char cur_path[SD_DIR_LENGTH] = { 0 };
        memset(cur_path, 0, SD_DIR_LENGTH);
        memcpy(cur_path, root, strlen(root));
        if (!(root[strlen(root) - 1] == '/')) strcat(cur_path, "/");
        strcat(cur_path, tmp.name());

        enumerate_files(cur_path, r, tabs + 1);

        // iterate tmp
        tmp = dir.openNextFile();
    }
    tmp.close();
    dir.close();
}
