#pragma once
#include <Arduino.h>
#include <stdarg.h>

/// @brief comment define statement to stop printing to serial monitor
#define LOGGER_FLAG

/// @brief size of internal buffer
#define LOGGER_BUFFER_SIZE 4096

enum class LogDestination {
    Serial,
    Comms
};

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

/// @brief Wrapper for storing print data from Serial
/// @note This class looks like a mess. It reimplements most of the things in the print class, but with the added ability to specifiy a destination.
/// @note The reason for the duplicate function declaration is: printf() uses variadic arguments, so we can't put LogDestinatoin last. And to default LogDestination anywhere else, we cant put it first. Therefore, we must overload instead of using default arguments.
class Logger {
public:
    /// @brief Default Constructor
    Logger() : cursor(0), write_error(0) { }
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
    /// @brief The main write(). Copies a buffer to the logger's internal buffer
    /// @return size_t of bytes written
    /// @param buffer Pointer to the input buffer
    /// @param size Size of the input buffer
    size_t write(const uint8_t* buffer, size_t size, LogDestination dest);

    /// @brief If write is just given a single string, recall with a casted uint8_t* buffer
    /// @param str String to write
    /// @param dest Destination to write to
    /// @return Returns the size_t written
    size_t write(const char* str, LogDestination dest) {
        if (str == nullptr) return 0;
        return write((const uint8_t*)str, strlen(str), dest);
    }

    /// @brief Write given a single byte
    /// @param b The byte to write
    /// @param dest Destination to write to
    /// @return Returns the size_t written
    size_t write(uint8_t b, LogDestination dest) {
        return write(&b, 1, dest);
    }

    /// @brief If write is given a const char*, recall write with it casted to a const uint8_t*
    size_t write(const char* buffer, size_t size, LogDestination dest) {
        return write((const uint8_t*)buffer, size, dest);
    }

    /// @brief amount of bytes currently stored in log
    /// @note also used as current position in memory
    uint32_t cursor;

// Stuff from teensy4/print.h
public:
    // print() section --------------------------------------------------------

    /// @brief Print a String (the class) to a specified destination.
    /// @param dest Destination to print to
    /// @param s The String to print
    /// @return size_t of bytes written
    size_t print(LogDestination dest, const String& s);
    /// @brief Print a String (the class) to the default destination (Comms)
    /// @param s The String to print
    /// @return size_t of bytes written
    size_t print(const String& s) { return print(LogDestination::Comms, s); }

    /// @brief Print a single character to a specified destination
    /// @param dest Destination to print to
    /// @param c The char to print
    /// @return size_t of bytes written
    size_t print(LogDestination dest, char c) { return write((uint8_t)c, dest); }
    /// @brief Print a single character to the default destination (Comms)
    /// @param c The char to print
    /// @return size_t of bytes written
    size_t print(char c) { return print(LogDestination::Comms, c); }

    /// @brief Print a string (not String class)
    /// @param dest Destination to print to
    /// @param s The string to print
    /// @return size_t of bytes written
    size_t print(LogDestination dest, const char s[ ]) { return write(s, dest); }
    /// @brief Print a string (not String class) to the default destination (Comms)
    /// @param s The string to print
    /// @return size_t of bytes written
    size_t print(const char s[ ]) { return print(LogDestination::Comms, s); }

    // Printing unsigned numbers ----------------

    /// @brief Print a single byte to a specified destination
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param b The byte to print
    /// @return size_t of bytes written
    size_t print(LogDestination dest, uint8_t b) { return printNumber(dest, b, 10, 0); }
    /// @brief Print a single byte to the default destination (Comms)
    /// @param b The byte to print
    /// @return size_t of bytes written
    size_t print(uint8_t b) { return print(LogDestination::Comms, b); }

    /// @brief 
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n 
    /// @return size_t of bytes written
    size_t print(LogDestination dest, unsigned int n) { return printNumber(dest, n, 10, 0); }
    /// @brief 
    /// @param n 
    /// @return size_t of bytes written
    size_t print(unsigned int n) { return print(LogDestination::Comms, n); }

    /// @brief Print an unsigned long to a specified destination
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n The unsigned long to print
    /// @return size_t of bytes written
    size_t print(LogDestination dest, unsigned long n) { return printNumber(dest, n, 10, 0); }
    /// @brief Print an unsigned long to the default destination (comms)
    /// @param n The unsigned long to print
    /// @return size_t of bytes written
    size_t print(unsigned long n) { return print(LogDestination::Comms, n); }

    /// @brief Print an unsigned long long to a specified destination
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n The unsigned long long to print
    /// @return size_t of bytes written
    size_t print(LogDestination dest, uint64_t n) { return printNumber64(dest, n, 10, 0); }
    /// @brief Print an unsigned long long to the default destination (comms)
    /// @param n The unsigned long long to print
    /// @return size_t of bytes written
    size_t print(uint64_t n) { return print(LogDestination::Comms, n); }

    /// @brief Print a number in any base (unsigned char) to a specified destination
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n The number to print
    /// @param base The base to print in
    /// @return size_t of bytes written
    size_t print(LogDestination dest, unsigned char n, int base) { return printNumber(dest, n, base, 0); }
    /// @brief Print a number in any base to the default destination (comms)
    /// @param n The number to print
    /// @param base The base to print in
    /// @return size_t of bytes written
    size_t print(unsigned char n, int base) { return print(LogDestination::Comms, n, base); }

    /// @brief Print an unsigned int in any base to a specified destination
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n The number to print
    /// @param base The base to print in
    /// @return size_t of bytes written
    size_t print(LogDestination dest, unsigned int n, int base) { return printNumber(dest, n, base, 0); }
    /// @brief Print an unsigned int in any base to the default destination (comms)
    /// @param n The number to print
    /// @param base The base to print in
    /// @return size_t of bytes written
    size_t print(unsigned int n, int base) { return print(LogDestination::Comms, n, base); }

    /// @brief Print an unsigned long in any base to a specified destination
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n The number to print
    /// @param base The base to print in
    /// @return size_t of bytes written
    size_t print(LogDestination dest, unsigned long n, int base) { return printNumber(dest, n, base, 0); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return size_t of bytes written
    size_t print(unsigned long n, int base) { return print(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n 
    /// @param base 
    /// @return size_t of bytes written
    size_t print(LogDestination dest, uint64_t n, int base) { return printNumber64(dest, n, base, 0); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return size_t of bytes written
    size_t print(uint64_t n, int base) { return print(LogDestination::Comms, n, base); }

    // Printing signed numbers ----------------------------------------------------------------

    /// @brief 
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n 
    /// @return size_t of bytes written
    size_t print(LogDestination dest, int n) { return print(dest, (long)n); }
    /// @brief 
    /// @param n 
    /// @return size_t of bytes written
    size_t print(int n) { return print(LogDestination::Comms, n); }

    /// @brief 
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n 
    /// @return size_t of bytes written
    size_t print(LogDestination dest, long n);
    /// @brief 
    /// @param n 
    /// @return size_t of bytes written
    size_t print(long n);

    /// @brief 
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n 
    /// @return size_t of bytes written
    size_t print(LogDestination dest, int64_t n);
    /// @brief 
    /// @param n 
    /// @return size_t of bytes written
    size_t print(int64_t n);

    /// @brief 
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n 
    /// @param base 
    /// @return size_t of bytes written
    size_t print(LogDestination dest, int n, int base) { return (base == 10) ? print(dest, n) : printNumber(dest, n, base, 0); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return size_t of bytes written
    size_t print(int n, int base) { return print(LogDestination::Comms, n, base); }


    /// @brief 
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n 
    /// @param base 
    /// @return size_t of bytes written
    size_t print(LogDestination dest, long n, int base) { return (base == 10) ? print(n) : printNumber(dest, n, base, 0); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return size_t of bytes written
    size_t print(long n, int base) { return print(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n 
    /// @param base 
    /// @return size_t of bytes written
    size_t print(LogDestination dest, int64_t n, int base) { return (base == 10) ? print(n) : printNumber64(dest, n, base, 0); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return size_t of bytes written
    size_t print(int64_t n, int base) { return print(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest Destination to print to. (See enum class LogDestination in logger.hpp. ex. LogDestination::Serial for serial.)
    /// @param n 
    /// @param digits 
    /// @return size_t of bytes written
    size_t print(LogDestination dest, double n, int digits = 2) { return printFloat(dest, n, digits); }
    /// @brief 
    /// @param n 
    /// @param digits 
    /// @return size_t of bytes written
    size_t print(double n, int digits = 2) { return print(LogDestination::Comms, n, digits); }

    // TODO: Print an object instance in human readable format
    // size_t print(const Printable& obj) { return obj.printTo(*this); }

    /// println section -------------------------------------------------------

    /// @brief Print a newline to a specified destination
    /// @param dest 
    /// @return 
    size_t println(LogDestination dest);
    /// @brief Print a newline to the default destination (comms)
    /// @return 
    size_t println() { return println(LogDestination::Comms); }

    /// @brief 
    /// @param dest 
    /// @param s 
    /// @return 
    size_t println(LogDestination dest, const String& s) { return print(dest, s) + println(dest); }
    /// @brief 
    /// @param s 
    /// @return 
    size_t println(const String& s) { return println(LogDestination::Comms, s); }

    /// @brief 
    /// @param dest 
    /// @param c 
    /// @return 
    size_t println(LogDestination dest, char c) { return print(dest, c) + println(dest); }
    /// @brief 
    /// @param c 
    /// @return 
    size_t println(char c) { return println(LogDestination::Comms, c); }

    /// @brief Print a C string
    /// @param dest 
    /// @param s 
    /// @return 
    size_t println(LogDestination dest, const char s[ ]) { return print(dest, s) + println(dest); }
    /// @brief 
    /// @param s 
    /// @return 
    size_t println(const char s[ ]) { return println(LogDestination::Comms, s); }

    /// @brief 
    /// @param dest 
    /// @param b 
    /// @return 
    size_t println(LogDestination dest, uint8_t b) { return print(dest, b) + println(dest); }
    /// @brief 
    /// @param b 
    /// @return 
    size_t println(uint8_t b) { return println(LogDestination::Comms, b); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @return 
    size_t println(LogDestination dest, int n) { return print(dest, n) + println(dest); }
    /// @brief 
    /// @param n 
    /// @return 
    size_t println(int n) { return println(LogDestination::Comms, n); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @return 
    size_t println(LogDestination dest, unsigned int n) { return print(dest, n) + println(dest); }
    /// @brief 
    /// @param n 
    /// @return 
    size_t println(unsigned int n) { return println(LogDestination::Comms, n); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @return 
    size_t println(LogDestination dest, long n) { return print(dest, n) + println(dest); }
    /// @brief 
    /// @param n 
    /// @return 
    size_t println(long n) { return println(LogDestination::Comms, n); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @return 
    size_t println(LogDestination dest, unsigned long n) { return print(dest, n) + println(dest); }
    /// @brief 
    /// @param n 
    /// @return 
    size_t println(unsigned long n) { return println(LogDestination::Comms, n); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @return 
    size_t println(LogDestination dest, int64_t n) { return print(dest, n) + println(dest); }
    /// @brief 
    /// @param n 
    /// @return 
    size_t println(int64_t n) { return println(LogDestination::Comms, n); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @return 
    size_t println(LogDestination dest, uint64_t n) { return print(dest, n) + println(dest); }
    /// @brief 
    /// @param n 
    /// @return 
    size_t println(uint64_t n) { return println(LogDestination::Comms, n); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(LogDestination dest, unsigned char n, int base) { return print(dest, n, base) + println(dest); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(unsigned char n, int base) { return println(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(LogDestination dest, int n, int base) { return print(dest, n, base) + println(dest); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(int n, int base) { return println(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(LogDestination dest, unsigned int n, int base) { return print(dest, n, base) + println(dest); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(unsigned int n, int base) { return println(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(LogDestination dest, long n, int base) { return print(dest, n, base) + println(dest); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(long n, int base) { return println(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(LogDestination dest, unsigned long n, int base) { return print(dest, n, base) + println(dest); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(unsigned long n, int base) { return println(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(LogDestination dest, int64_t n, int base) { return print(dest, n, base) + println(dest); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(int64_t n, int base) { return println(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(LogDestination dest, uint64_t n, int base) { return print(dest, n, base) + println(dest); }
    /// @brief 
    /// @param n 
    /// @param base 
    /// @return 
    size_t println(uint64_t n, int base) { return println(LogDestination::Comms, n, base); }

    /// @brief 
    /// @param dest 
    /// @param n 
    /// @param digits 
    /// @return 
    size_t println(LogDestination dest, double n, int digits = 2) { return print(dest, n, digits) + println(dest); }
    /// @brief 
    /// @param n 
    /// @param digits 
    /// @return 
    size_t println(double n, int digits = 2) { return println(LogDestination::Comms, n, digits); }

        // Print an object instance in human readable format, and a newline
        // size_t println(const Printable& obj) { return obj.printTo(*this) + println(); }



    int getwriteError() { return write_error; }
    void clearwriteError() { setwriteError(0); }

    // printf is a C standard function which allows you to print any number of variables using a somewhat cryptic format string
    int printf(LogDestination dest, const char* format, ...);

    int printf(const char* format, ...);
    // printf is a C standard function which allows you to print any number of variables using a somewhat cryptic format string
    // int printf(const __FlashStringHelper* format, LogDestination dest = LogDestination::Comms,  ...);
    // vprintf is a C standard function that allows you to print a variable argument list with a format string
    int vprintf(LogDestination dest, const char* format, va_list ap);

    // format warnings are too pedantic - disable until newer toolchain offers better...
    // https://forum.pjrc.com/threads/62473?p=256873&viewfull=1#post256873
    // int printf(const char *format, ...) __attribute__ ((format (printf, 2, 3)));

protected:
    void setwriteError(int err = 1) { write_error = err; }
private:
    int write_error;
    size_t printFloat(LogDestination dest, double n, uint8_t digits);
    size_t printNumber(LogDestination dest, unsigned long n, uint8_t base, uint8_t sign);
    size_t printNumber64(LogDestination dest, uint64_t n, uint8_t base, uint8_t sign);
};

/// @brief universal logger object to be called anywhere in codebase
inline Logger logger;
