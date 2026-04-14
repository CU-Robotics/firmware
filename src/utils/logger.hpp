#pragma once
#include <Arduino.h>
#include <stdarg.h>

/// @brief Define this flag to enable logger functionality
#define LOGGER_FLAG

/// @brief Size of internal buffer
#define LOGGER_BUFFER_SIZE 4096

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

/// @brief Wrapper for storing print data from Serial.
/// @note This class mirrors Serial's print-style API while also copying all
/// output into the comms log buffer.
class Logger {
  public:
    /// @brief Default Constructor.
    Logger();
    /// @brief Default Destructor.
    ~Logger() = default;

    /// @brief Copies the internal buffer to the provided memory location.
    /// @param size Number of bytes to copy.
    /// @param dest Pointer to the buffer where log data will be copied.
    /// @return Number of bytes copied.
    size_t grab_log_data(size_t size, uint8_t *dest);

  private:
    /// @brief Current number of bytes stored in the log (also serves as the
    /// current buffer position).
    size_t cursor;

    /// @brief Main write function that copies a buffer to serial and the internal log.
    /// @param buffer Pointer to the input buffer.
    /// @param size Size of the input buffer.
    /// @return Number of bytes written.
    size_t write(const uint8_t *buffer, size_t size);

    /// @brief Writes a null-terminated string.
    /// @param str String to write.
    /// @return Number of bytes written.
    size_t write(const char *str) {
        if (str == nullptr)
            return 0;
        return write((const uint8_t *)str, strlen(str));
    }

    /// @brief Writes a single byte.
    /// @param b Byte to write.
    /// @return Number of bytes written.
    size_t write(uint8_t b) { return write(&b, 1); }

    /// @brief Writes a character buffer (not necessarily null-terminated).
    /// @param buffer Pointer to the character buffer.
    /// @param size Number of characters in the buffer.
    /// @return Number of bytes written.
    size_t write(const char *buffer, size_t size) { return write((const uint8_t *)buffer, size); }

    // -------- formatting helpers ---------------------------------------------

    /// @brief Prints a floating point number with fixed precision.
    /// @param n Floating point number to print.
    /// @param digits Precision, number of digits after the decimal point.
    /// @return Number of bytes written.
    size_t printFloat(double n, uint8_t digits);

    /// @brief Prints an unsigned number in a given base.
    /// @param n Unsigned number to print.
    /// @param base Numerical base for printing.
    /// @param sign Sign (+/-)
    /// @return Number of bytes written.
    size_t printNumber(unsigned long n, uint8_t base, uint8_t sign);

    /// @brief Prints a 64-bit unsigned number in a given base.
    /// @param n 64-bit unsigned number to print.
    /// @param base Numerical base for printing.
    /// @param sign Sign (+/-)
    /// @return Number of bytes written.
    size_t printNumber64(uint64_t n, uint8_t base, uint8_t sign);

  public:
    /// @cond // doxygen ignore, these are explained by their params
    // -------- print() strings and chars --------------------------------------

    size_t print(const String &s);
    size_t print(const char s[]);
    size_t print(char c);
    size_t print(unsigned char n, int base);

    size_t println();
    size_t println(const String &s);
    size_t println(const char s[]);
    size_t println(char c);
    size_t println(unsigned char n, int base);

    // TODO: flash string support

    // TODO: object printing
    // size_t print(const Printable& obj) { return obj.printTo(*this); }

    // --------- print() unsigned numbers -------------------------------------

    size_t print(uint8_t b);
    size_t print(unsigned int n);
    size_t print(unsigned long n);
    size_t print(uint64_t n);
    size_t print(unsigned int n, int base);
    size_t print(unsigned long n, int base);
    size_t print(uint64_t n, int base);

    size_t println(uint8_t b);
    size_t println(unsigned int n);
    size_t println(unsigned long n);
    size_t println(uint64_t n);
    size_t println(unsigned int n, int base);
    size_t println(unsigned long n, int base);
    size_t println(uint64_t n, int base);

    // -------- print() signed numbers -----------------------------------------

    size_t print(int n);
    size_t print(long n);
    size_t print(int64_t n);
    size_t print(int n, int base);
    size_t print(long n, int base);
    size_t print(int64_t n, int base);
    size_t print(double n, int digits = 2);

    size_t println(int n);
    size_t println(long n);
    size_t println(int64_t n);
    size_t println(int n, int base);
    size_t println(long n, int base);
    size_t println(int64_t n, int base);
    size_t println(double n, int digits = 2);

    // -------- printf() -------------------------------------------------------

    int printf(const char *format, ...);
    int vprintf(const char *format, va_list ap);

    /// @endcond // end doxygen ignore

    /// @brief Retrieves the current write error code.
    /// @return Current write error code.
    int getWriteError() { return write_error; }
    /// @brief Clears the current write error.
    void clearWriteError() { setWriteError(0); }

  protected:
    /// @brief Sets the write error code.
    /// @param err Error code to set (default is 1).
    void setWriteError(int err = 1) { write_error = err; }

  private:
    /// @brief Write error code.
    int write_error;
};

/// @brief Universal logger object accessible anywhere in the codebase.
inline Logger logger;
