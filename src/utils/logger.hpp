#pragma once
#include <Arduino.h>
#include <stdarg.h>

/// @brief comment define statement to stop printing to serial monitor
#define LOGGER_FLAG

/// @brief size of internal buffer
#define LOGGER_BUFFER_SIZE 4096

enum class LogDestination {
    Serial,
    File
};

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

/// @brief Wrapper for storing print data from Serial
class Logger {
public:
    /// @brief Default Constructor
    Logger() : write_error(0) { }
    /// @brief Default Destructor
    ~Logger() = default;

    /// @brief copies internal buffer to inputted location (*data) in memory
    /// @return number of bytes copied
    /// @param size of data
    /// @param data pointer to data
    uint32_t grab_log_data(uint32_t size, uint8_t* data);

    /// @brief Implemented to appease the virtual write 
    /// @param b Arguments match virtual write
    /// @return not much

private:
    /// @brief copies formatted bytes to internal buffer
    /// @return number of bytes
    /// @param buffer pointer to a buffer
    /// @param size size of the buffer
    size_t write(const uint8_t* buffer, size_t size, LogDestination dest);

    size_t write(uint8_t b, LogDestination dest) {
        return write(&b, 1, dest);
    }

    /// @brief If write is just given a single string, recall with a casted uint8_t* buffer
    /// @param str String to write
    /// @param dest destination to write to
    /// @return Returns the result of the nested write.
    size_t write(const char* str, LogDestination dest) {
        if (str == nullptr) return 0;
        return write((const uint8_t*)str, strlen(str), dest);
    }


    /// @brief amount of bytes currently stored in log
    /// @note also used as current position in memory
    uint32_t cursor;

// Stuff from teensy4/print.h
public:
    virtual int availableForwrite(void) { return 0; }
    virtual void flush() { }
    // Print a string
    size_t print(const String& s, LogDestination dest = LogDestination::Serial);
    // Print a single character
    size_t print(char c, LogDestination dest = LogDestination::Serial) { return write((uint8_t)c, dest); }
    // Print a string
    size_t print(const char s[ ], LogDestination dest = LogDestination::Serial) { return write(s, dest); }
    // Print an unsigned number
    size_t print(uint8_t b, LogDestination dest = LogDestination::Serial) { return printNumber(b, 10, 0, dest); }
    // Print a signed number
    size_t print(int n, LogDestination dest = LogDestination::Serial) { return print((long)n, dest); }
    // Print an unsigned number
    size_t print(unsigned int n, LogDestination dest = LogDestination::Serial) { return printNumber(n, 10, 0, dest); }
    // Print a signed number
    size_t print(long n, LogDestination dest = LogDestination::Serial);
    // Print an unsigned number
    size_t print(unsigned long n, LogDestination dest = LogDestination::Serial) { return printNumber(n, 10, 0, dest); }
    // Print a signed number
    size_t print(int64_t n, LogDestination dest = LogDestination::Serial);
    // Print an unsigned number
    size_t print(uint64_t n, LogDestination dest = LogDestination::Serial) { return printNumber64(n, 10, 0, dest); }

    // Print a number in any number base (eg, BIN, HEX, OCT)
    size_t print(unsigned char n, int base, LogDestination dest = LogDestination::Serial) { return printNumber(n, base, 0, dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT)
    size_t print(int n, int base, LogDestination dest = LogDestination::Serial) { return (base == 10) ? print(n, dest) : printNumber(n, base, 0, dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT)
    size_t print(unsigned int n, int base, LogDestination dest = LogDestination::Serial) { return printNumber(n, base, 0, dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT)
    size_t print(long n, int base, LogDestination dest = LogDestination::Serial) { return (base == 10) ? print(n) : printNumber(n, base, 0, dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT)
    size_t print(unsigned long n, int base, LogDestination dest = LogDestination::Serial) { return printNumber(n, base, 0, dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT)
    size_t print(int64_t n, int base, LogDestination dest = LogDestination::Serial) { return (base == 10) ? print(n) : printNumber64(n, base, 0, dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT)
    size_t print(uint64_t n, int base, LogDestination dest = LogDestination::Serial) { return printNumber64(n, base, 0, dest); }

    // Print a floating point (decimal) number
    size_t print(double n, int digits = 2, LogDestination dest = LogDestination::Serial) { return printFloat(n, digits, dest); }
    // Print an object instance in human readable format
    // size_t print(const Printable& obj) { return obj.printTo(*this); }
    // Print a newline
    size_t println(LogDestination dest = LogDestination::Serial);
    // Print a string and newline
    size_t println(const String& s, LogDestination dest = LogDestination::Serial) { return print(s, dest) + println(dest); }
    // Print a single character and newline
    size_t println(char c, LogDestination dest = LogDestination::Serial) { return print(c, dest) + println(dest); }
    // Print a string and newline
    size_t println(const char s[ ], LogDestination dest = LogDestination::Serial) { return print(s, dest) + println(dest); }
    // Print a string and newline
    // size_t println(const __FlashStringHelper* f) { return print(f) + println(); }

    // Print an unsigned number and newline
    size_t println(uint8_t b, LogDestination dest = LogDestination::Serial) { return print(b, dest) + println(dest); }
    // Print a signed number and newline
    size_t println(int n, LogDestination dest = LogDestination::Serial) { return print(n, dest) + println(dest); }
    // Print an unsigned number and newline
    size_t println(unsigned int n, LogDestination dest = LogDestination::Serial) { return print(n, dest) + println(dest); }
    // Print a signed number and newline
    size_t println(long n, LogDestination dest = LogDestination::Serial) { return print(n, dest) + println(dest); }
    // Print an unsigned number and newline
    size_t println(unsigned long n, LogDestination dest = LogDestination::Serial) { return print(n, dest) + println(dest); }
    // Print a signed number and newline
    size_t println(int64_t n, LogDestination dest = LogDestination::Serial) { return print(n, dest) + println(dest); }
    // Print an unsigned number and newline
    size_t println(uint64_t n, LogDestination dest = LogDestination::Serial) { return print(n, dest) + println(dest); }

    // Print a number in any number base (eg, BIN, HEX, OCT) and a newline
    size_t println(unsigned char n, int base, LogDestination dest = LogDestination::Serial) { return print(n, base, dest) + println(dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT) and a newline
    size_t println(int n, int base, LogDestination dest = LogDestination::Serial) { return print(n, base, dest) + println(dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT) and a newline
    size_t println(unsigned int n, int base, LogDestination dest = LogDestination::Serial) { return print(n, base, dest) + println(dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT) and a newline
    size_t println(long n, int base, LogDestination dest = LogDestination::Serial) { return print(n, base, dest) + println(dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT) and a newline
    size_t println(unsigned long n, int base, LogDestination dest = LogDestination::Serial) { return print(n, base, dest) + println(dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT) and a newline
    size_t println(int64_t n, int base, LogDestination dest = LogDestination::Serial) { return print(n, base, dest) + println(dest); }
    // Print a number in any number base (eg, BIN, HEX, OCT) and a newline
    size_t println(uint64_t n, int base, LogDestination dest = LogDestination::Serial) { return print(n, base, dest) + println(dest); }

    // Print a floating point (decimal) number and a newline
    size_t println(double n, int digits = 2, LogDestination dest = LogDestination::Serial) { return print(n, digits, dest) + println(dest); }
    // Print an object instance in human readable format, and a newline
    // size_t println(const Printable& obj) { return obj.printTo(*this) + println(); }
    int getwriteError() { return write_error; }
    void clearwriteError() { setwriteError(0); }

    // printf is a C standard function which allows you to print any number of variables using a somewhat cryptic format string
    int printf(LogDestination dest, const char* format, ...);

    int printf(const char* format, ...);
    // printf is a C standard function which allows you to print any number of variables using a somewhat cryptic format string
    // int printf(const __FlashStringHelper* format, LogDestination dest = LogDestination::Serial,  ...);
    // vprintf is a C standard function that allows you to print a variable argument list with a format string
    int vprintf(LogDestination dest, const char* format, va_list ap);

    // format warnings are too pedantic - disable until newer toolchain offers better...
    // https://forum.pjrc.com/threads/62473?p=256873&viewfull=1#post256873
    // int printf(const char *format, ...) __attribute__ ((format (printf, 2, 3)));

protected:
    void setwriteError(int err = 1) { write_error = err; }
private:
    int write_error;
    size_t printFloat(double n, uint8_t digits, LogDestination dest);
    size_t printNumber(unsigned long n, uint8_t base, uint8_t sign, LogDestination dest);
    size_t printNumber64(uint64_t n, uint8_t base, uint8_t sign, LogDestination dest);
};

/// @brief universal logger object to be called anywhere in codebase
inline Logger logger;
