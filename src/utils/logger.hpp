#pragma once
#include <Arduino.h>
#include <stdarg.h>

/// @brief Comment define statement to stop printing to serial monitor
#define LOGGER_FLAG

/// @brief Size of internal buffer
#define LOGGER_BUFFER_SIZE 4096

enum class LogDestination {
    Serial,
    Comms
};

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

/// @brief Wrapper for storing print data from Serial.
/// @note This class reimplements many functions from the Print class with the added ability to specify a destination.
/// @note Due to variadic arguments in printf(), overloads are used instead of default parameters.
class Logger {
public:
    /// @brief Default Constructor.
    Logger();
    /// @brief Default Destructor.
    ~Logger() = default;

    /// @brief Copies the internal buffer to the provided memory location.
    /// @param size Number of bytes to copy.
    /// @param data Pointer to the buffer where log data will be copied.
    /// @return Number of bytes copied.
    uint32_t grab_log_data(uint32_t size, uint8_t* data);

private:
    /// @brief Main write function that copies a buffer to the internal log.
    /// @param buffer Pointer to the input buffer.
    /// @param size Size of the input buffer.
    /// @param dest Destination for the log output.
    /// @return Number of bytes written.
    size_t write(const uint8_t* buffer, size_t size, LogDestination dest);

    /// @brief Writes a null-terminated string to the specified destination.
    /// @param str String to write.
    /// @param dest Destination for the log output.
    /// @return Number of bytes written.
    size_t write(const char* str, LogDestination dest) {
        if (str == nullptr) return 0;
        return write((const uint8_t*)str, strlen(str), dest);
    }

    /// @brief Writes a single byte to the specified destination.
    /// @param b Byte to write.
    /// @param dest Destination for the log output.
    /// @return Number of bytes written.
    size_t write(uint8_t b, LogDestination dest) {
        return write(&b, 1, dest);
    }

    /// @brief Writes a character buffer (not necessarily null-terminated) to the specified destination.
    /// @param buffer Pointer to the character buffer.
    /// @param size Number of characters in the buffer.
    /// @param dest Destination for the log output.
    /// @return Number of bytes written.
    size_t write(const char* buffer, size_t size, LogDestination dest) {
        return write((const uint8_t*)buffer, size, dest);
    }

    /// @brief Current number of bytes stored in the log (also serves as the current buffer position).
    uint32_t cursor;

public:
    // print() section --------------------------------------------------------

    /// @brief Prints a String object to a specified destination.
    /// @param dest Destination for the output.
    /// @param s String to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, const String& s);
    /// @brief Prints a String object to the default destination (Comms).
    /// @param s String to print.
    /// @return Number of bytes written.
    size_t print(const String& s);

    /// @brief Prints a single character to a specified destination.
    /// @param dest Destination for the output.
    /// @param c Character to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, char c);
    /// @brief Prints a single character to the default destination (Comms).
    /// @param c Character to print.
    /// @return Number of bytes written.
    size_t print(char c);

    /// @brief Prints a C-string to a specified destination.
    /// @param dest Destination for the output.
    /// @param s C-string to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, const char s[ ]);
    /// @brief Prints a C-string to the default destination (Comms).
    /// @param s C-string to print.
    /// @return Number of bytes written.
    size_t print(const char s[ ]);

    // Printing unsigned numbers ----------------

    /// @brief Prints an unsigned byte (in decimal) to a specified destination.
    /// @param dest Destination for the output.
    /// @param b Unsigned byte to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, uint8_t b);
    /// @brief Prints an unsigned byte (in decimal) to the default destination (Comms).
    /// @param b Unsigned byte to print.
    /// @return Number of bytes written.
    size_t print(uint8_t b);

    /// @brief Prints an unsigned integer (in decimal) to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned integer to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, unsigned int n);
    /// @brief Prints an unsigned integer (in decimal) to the default destination (Comms).
    /// @param n Unsigned integer to print.
    /// @return Number of bytes written.
    size_t print(unsigned int n);

    /// @brief Prints an unsigned long (in decimal) to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned long to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, unsigned long n);
    /// @brief Prints an unsigned long (in decimal) to the default destination (Comms).
    /// @param n Unsigned long to print.
    /// @return Number of bytes written.
    size_t print(unsigned long n);

    /// @brief Prints an unsigned long long (in decimal) to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned long long to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, uint64_t n);
    /// @brief Prints an unsigned long long (in decimal) to the default destination (Comms).
    /// @param n Unsigned long long to print.
    /// @return Number of bytes written.
    size_t print(uint64_t n);

    /// @brief Prints an unsigned byte in a specified base to a destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned byte to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, unsigned char n, int base);
    /// @brief Prints an unsigned byte in a specified base to the default destination (Comms).
    /// @param n Unsigned byte to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(unsigned char n, int base);

    /// @brief Prints an unsigned integer in a specified base to a destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, unsigned int n, int base);
    /// @brief Prints an unsigned integer in a specified base to the default destination (Comms).
    /// @param n Unsigned integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(unsigned int n, int base);

    /// @brief Prints an unsigned long in a specified base to a destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned long to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, unsigned long n, int base);
    /// @brief Prints an unsigned long in a specified base to the default destination (Comms).
    /// @param n Unsigned long to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(unsigned long n, int base);

    /// @brief Prints an unsigned long long in a specified base to a destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned long long to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, uint64_t n, int base);
    /// @brief Prints an unsigned long long in a specified base to the default destination (Comms).
    /// @param n Unsigned long long to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(uint64_t n, int base);

    // Printing signed numbers ----------------------------------------------------------------

    /// @brief Prints a signed integer to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Signed integer to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, int n);
    /// @brief Prints a signed integer to the default destination (Comms).
    /// @param n Signed integer to print.
    /// @return Number of bytes written.
    size_t print(int n);

    /// @brief Prints a long integer to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Long integer to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, long n);
    /// @brief Prints a long integer to the default destination (Comms).
    /// @param n Long integer to print.
    /// @return Number of bytes written.
    size_t print(long n);

    /// @brief Prints a 64-bit signed integer to a specified destination.
    /// @param dest Destination for the output.
    /// @param n 64-bit signed integer to print.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, int64_t n);
    /// @brief Prints a 64-bit signed integer to the default destination (Comms).
    /// @param n 64-bit signed integer to print.
    /// @return Number of bytes written.
    size_t print(int64_t n);

    /// @brief Prints a signed integer in a specified base to a destination.
    /// @param dest Destination for the output.
    /// @param n Signed integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, int n, int base);
    /// @brief Prints a signed integer in a specified base to the default destination (Comms).
    /// @param n Signed integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(int n, int base);

    /// @brief Prints a long integer in a specified base to a destination.
    /// @param dest Destination for the output.
    /// @param n Long integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, long n, int base);
    /// @brief Prints a long integer in a specified base to the default destination (Comms).
    /// @param n Long integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(long n, int base);

    /// @brief Prints a 64-bit signed integer in a specified base to a destination.
    /// @param dest Destination for the output.
    /// @param n 64-bit signed integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, int64_t n, int base);
    /// @brief Prints a 64-bit signed integer in a specified base to the default destination (Comms).
    /// @param n 64-bit signed integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t print(int64_t n, int base);

    /// @brief Prints a floating point number to a specified destination with fixed precision.
    /// @param dest Destination for the output.
    /// @param n Floating point number to print.
    /// @param digits Number of digits after the decimal point.
    /// @return Number of bytes written.
    size_t print(LogDestination dest, double n, int digits = 2);
    /// @brief Prints a floating point number to the default destination (Comms) with fixed precision.
    /// @param n Floating point number to print.
    /// @param digits Number of digits after the decimal point.
    /// @return Number of bytes written.
    size_t print(double n, int digits = 2);

    // TODO: Print an object instance in human readable format
    // size_t print(const Printable& obj) { return obj.printTo(*this); }

    // println section -------------------------------------------------------

    /// @brief Prints a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @return Number of bytes written.
    size_t println(LogDestination dest);
    /// @brief Prints a newline to the default destination (Comms).
    /// @return Number of bytes written.
    size_t println();

    /// @brief Prints a String followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param s String to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, const String& s);
    /// @brief Prints a String followed by a newline to the default destination (Comms).
    /// @param s String to print.
    /// @return Number of bytes written.
    size_t println(const String& s);

    /// @brief Prints a character followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param c Character to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, char c);
    /// @brief Prints a character followed by a newline to the default destination (Comms).
    /// @param c Character to print.
    /// @return Number of bytes written.
    size_t println(char c);

    /// @brief Prints a C-string followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param s C-string to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, const char s[ ]);
    /// @brief Prints a C-string followed by a newline to the default destination (Comms).
    /// @param s C-string to print.
    /// @return Number of bytes written.
    size_t println(const char s[ ]);

    /// @brief Prints an unsigned byte followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param b Unsigned byte to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, uint8_t b);
    /// @brief Prints an unsigned byte followed by a newline to the default destination (Comms).
    /// @param b Unsigned byte to print.
    /// @return Number of bytes written.
    size_t println(uint8_t b);

    /// @brief Prints a signed integer followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Signed integer to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, int n);
    /// @brief Prints a signed integer followed by a newline to the default destination (Comms).
    /// @param n Signed integer to print.
    /// @return Number of bytes written.
    size_t println(int n);

    /// @brief Prints an unsigned integer followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned integer to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, unsigned int n);
    /// @brief Prints an unsigned integer followed by a newline to the default destination (Comms).
    /// @param n Unsigned integer to print.
    /// @return Number of bytes written.
    size_t println(unsigned int n);

    /// @brief Prints a long integer followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Long integer to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, long n);
    /// @brief Prints a long integer followed by a newline to the default destination (Comms).
    /// @param n Long integer to print.
    /// @return Number of bytes written.
    size_t println(long n);

    /// @brief Prints an unsigned long integer followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned long integer to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, unsigned long n);
    /// @brief Prints an unsigned long integer followed by a newline to the default destination (Comms).
    /// @param n Unsigned long integer to print.
    /// @return Number of bytes written.
    size_t println(unsigned long n);

    /// @brief Prints a 64-bit signed integer followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param n 64-bit signed integer to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, int64_t n);
    /// @brief Prints a 64-bit signed integer followed by a newline to the default destination (Comms).
    /// @param n 64-bit signed integer to print.
    /// @return Number of bytes written.
    size_t println(int64_t n);

    /// @brief Prints a 64-bit unsigned integer followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param n 64-bit unsigned integer to print.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, uint64_t n);
    /// @brief Prints a 64-bit unsigned integer followed by a newline to the default destination (Comms).
    /// @param n 64-bit unsigned integer to print.
    /// @return Number of bytes written.
    size_t println(uint64_t n);

    /// @brief Prints an unsigned byte in a specified base followed by a newline to a destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned byte to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, unsigned char n, int base);
    /// @brief Prints an unsigned byte in a specified base followed by a newline to the default destination (Comms).
    /// @param n Unsigned byte to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(unsigned char n, int base);

    /// @brief Prints an integer in a specified base followed by a newline to a destination.
    /// @param dest Destination for the output.
    /// @param n Integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, int n, int base);
    /// @brief Prints an integer in a specified base followed by a newline to the default destination (Comms).
    /// @param n Integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(int n, int base);

    /// @brief Prints an unsigned integer in a specified base followed by a newline to a destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, unsigned int n, int base);
    /// @brief Prints an unsigned integer in a specified base followed by a newline to the default destination (Comms).
    /// @param n Unsigned integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(unsigned int n, int base);

    /// @brief Prints a long integer in a specified base followed by a newline to a destination.
    /// @param dest Destination for the output.
    /// @param n Long integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, long n, int base);
    /// @brief Prints a long integer in a specified base followed by a newline to the default destination (Comms).
    /// @param n Long integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(long n, int base);

    /// @brief Prints an unsigned long in a specified base followed by a newline to a destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned long to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, unsigned long n, int base);
    /// @brief Prints an unsigned long in a specified base followed by a newline to the default destination (Comms).
    /// @param n Unsigned long to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(unsigned long n, int base);

    /// @brief Prints a 64-bit signed integer in a specified base followed by a newline to a destination.
    /// @param dest Destination for the output.
    /// @param n 64-bit signed integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, int64_t n, int base);
    /// @brief Prints a 64-bit signed integer in a specified base followed by a newline to the default destination (Comms).
    /// @param n 64-bit signed integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(int64_t n, int base);

    /// @brief Prints a 64-bit unsigned integer in a specified base followed by a newline to a destination.
    /// @param dest Destination for the output.
    /// @param n 64-bit unsigned integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, uint64_t n, int base);
    /// @brief Prints a 64-bit unsigned integer in a specified base followed by a newline to the default destination (Comms).
    /// @param n 64-bit unsigned integer to print.
    /// @param base Numerical base for printing.
    /// @return Number of bytes written.
    size_t println(uint64_t n, int base);

    /// @brief Prints a floating point number followed by a newline to a specified destination.
    /// @param dest Destination for the output.
    /// @param n Floating point number to print.
    /// @param digits Number of digits after the decimal point.
    /// @return Number of bytes written.
    size_t println(LogDestination dest, double n, int digits = 2);
    /// @brief Prints a floating point number followed by a newline to the default destination (Comms).
    /// @param n Floating point number to print.
    /// @param digits Number of digits after the decimal point.
    /// @return Number of bytes written.
    size_t println(double n, int digits = 2);

    /// @brief Retrieves the current write error code.
    /// @return Current write error code.
    int getwriteError() { return write_error; }
    /// @brief Clears the current write error.
    void clearwriteError() { setwriteError(0); }

    /// @brief Prints formatted data to a specified destination.
    /// @param dest Destination for the output.
    /// @param format Format string (printf style).
    /// @param ... Additional arguments.
    /// @return Number of characters printed.
    int printf(LogDestination dest, const char* format, ...);

    /// @brief Prints formatted data to the default destination (Comms).
    /// @param format Format string (printf style).
    /// @param ... Additional arguments.
    /// @return Number of characters printed.
    int printf(const char* format, ...);

    /// @brief Prints formatted data using a variable argument list to a specified destination.
    /// @param dest Destination for the output.
    /// @param format Format string (printf style).
    /// @param ap Variable argument list.
    /// @return Number of characters printed.
    int vprintf(LogDestination dest, const char* format, va_list ap);

protected:
    /// @brief Sets the write error code.
    /// @param err Error code to set (default is 1).
    void setwriteError(int err = 1) { write_error = err; }
private:
    /// @brief Write error code.
    int write_error;
    /// @brief Prints a floating point number with fixed precision to the specified destination.
    /// @param dest Destination for the output.
    /// @param n Floating point number to print.
    /// @param digits Number of digits after the decimal point.
    /// @return Number of bytes written.
    size_t printFloat(LogDestination dest, double n, uint8_t digits);
    /// @brief Prints an unsigned number in a given base to the specified destination.
    /// @param dest Destination for the output.
    /// @param n Unsigned number to print.
    /// @param base Numerical base for printing.
    /// @param sign The sign
    /// @return Number of bytes written.
    size_t printNumber(LogDestination dest, unsigned long n, uint8_t base, uint8_t sign);
    /// @brief Prints a 64-bit unsigned number in a given base to the specified destination.
    /// @param dest Destination for the output.
    /// @param n 64-bit unsigned number to print.
    /// @param base Numerical base for printing.
    /// @param sign The sign
    /// @return Number of bytes written.
    size_t printNumber64(LogDestination dest, uint64_t n, uint8_t base, uint8_t sign);
};

/// @brief Universal logger object accessible anywhere in the codebase.
inline Logger logger;
