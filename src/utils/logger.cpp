#include "logger.hpp"

#include "comms/comms_layer.hpp"
#include "comms/data/logging_data.hpp"
#include "comms/data/sendable.hpp"

/// @brief internal buffer with 4kb capacity
/// @note will need to change size if we every print more than 4096 characters
/// per loop
DMAMEM char log_buffer[LOGGER_BUFFER_SIZE];

// need to clear the buffer before using it
Logger::Logger() : cursor(0), write_error(0) {
    // Initialize the log buffer
    memset(log_buffer, 0, sizeof(log_buffer));
}

// -------- Logger internal functions -----------------------------------------

size_t Logger::write(const uint8_t* buffer, size_t size,
                     LogDestination destination) {
    // logger.printf(LogDestination::Serial, "%d %d\n", cursor, size);
    // guard against cursor going beyond buffer size
    if (cursor + size >= sizeof(log_buffer)) {
        return 0;
    }

    // writes incoming content (buffer) to cursor position
    memcpy(log_buffer + cursor, buffer, size);
    char print_statement[sizeof(log_buffer)] = {0};  // temp variable

    // print_statement captures only the most recent addition to log_buffer
    memcpy(print_statement, log_buffer + cursor, size);

    if (destination == LogDestination::Serial) {
        // print to serial monitor
        Serial.print(print_statement);
    }

    // sets cursor to current place in memory
    cursor += size;
    return size;
}

uint32_t Logger::grab_log_data(uint32_t size, uint8_t* dest) {
    // ensure dest isn't a null pointer
    if (dest == nullptr) {
        return 0;
    }

    // make sure we don't copy more than the buffer size
    uint32_t bytes_to_copy = std::min(cursor, size);

    // copy internal buffer into *dest
    memcpy(dest, log_buffer, bytes_to_copy);

    // reset cursor
    cursor = 0;

    // return number of bytes copied
    return bytes_to_copy;
}

// Function to handle the actual formatted printing
int Logger::vprintf(LogDestination dest, const char* format, va_list args) {
    uint8_t buffer[1024];
    int retval = vsnprintf_((char*)buffer, 1024, format, args);
    write(buffer, retval, dest);

    return retval;
}

// Default behavior: print to stdout
int Logger::printf(const char* format, ...) {
    va_list args;
    va_start(args, format);

    int retval = vprintf(LogDestination::Comms, format,
                         args);  // Default: print to stdout

    va_end(args);

    return retval;
}

// Overload with explicit log destination
int Logger::printf(LogDestination dest, const char* format, ...) {
    va_list args;
    va_start(args, format);

    int retval = vprintf(dest, format, args);

    va_end(args);

    return retval;
}

size_t Logger::printNumber(LogDestination dest, unsigned long n, uint8_t base,
                           uint8_t sign) {
    uint8_t buf[34];
    uint8_t digit, i;

    // TODO: make these checks as inline, since base is
    // almost always a constant.  base = 0 (BYTE) should
    // inline as a call directly to write()
    if (base == 0) {
        return write((uint8_t)n, dest);
    } else if (base == 1) {
        base = 10;
    }

    if (n == 0) {
        buf[sizeof(buf) - 1] = '0';
        i = sizeof(buf) - 1;
    } else {
        i = sizeof(buf) - 1;
        while (1) {
            digit = n % base;
            buf[i] = ((digit < 10) ? '0' + digit : 'A' + digit - 10);
            n /= base;
            if (n == 0) break;
            i--;
        }
    }
    if (sign) {
        i--;
        buf[i] = '-';
    }
    return write(buf + i, sizeof(buf) - i, dest);
}

size_t Logger::printNumber64(LogDestination dest, uint64_t n, uint8_t base,
                             uint8_t sign) {
    uint8_t buf[66];
    uint8_t digit, i;

    if (base < 2) return 0;
    if (n == 0) {
        buf[sizeof(buf) - 1] = '0';
        i = sizeof(buf) - 1;
    } else {
        i = sizeof(buf) - 1;
        while (1) {
            digit = n % base;
            buf[i] = ((digit < 10) ? '0' + digit : 'A' + digit - 10);
            n /= base;
            if (n == 0) break;
            i--;
        }
    }
    if (sign) {
        i--;
        buf[i] = '-';
    }
    return write(buf + i, sizeof(buf) - i, dest);
}

size_t Logger::printFloat(LogDestination dest, double number, uint8_t digits) {
    uint8_t sign = 0;
    size_t count = 0;

    if (isnan(number)) return print("nan");
    if (isinf(number)) return print("inf");
    if (number > 4294967040.0f)
        return print("ovf");  // constant determined empirically
    if (number < -4294967040.0f)
        return print("ovf");  // constant determined empirically

    // Handle negative numbers
    if (number < 0.0) {
        sign = 1;
        number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i) {
        rounding *= 0.1;
    }
    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    count += printNumber(dest, int_part, 10, sign);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        uint8_t n, buf[16], count = 1;
        buf[0] = '.';

        // Extract digits from the remainder one at a time
        if (digits > sizeof(buf) - 1) digits = sizeof(buf) - 1;

        while (digits-- > 0) {
            remainder *= 10.0;
            n = (uint8_t)(remainder);
            buf[count++] = '0' + n;
            remainder -= n;
        }
        count += write(buf, count, dest);
    }
    return count;
}

// -------- print() strings and chars -----------------------------------------

size_t Logger::print(LogDestination dest, const String& s) {
    uint8_t buffer[33];
    size_t count = 0;
    unsigned int index = 0;
    unsigned int len = s.length();
    while (len > 0) {
        s.getBytes(buffer, sizeof(buffer), index);
        unsigned int nbytes = len;
        if (nbytes > sizeof(buffer) - 1) nbytes = sizeof(buffer) - 1;
        index += nbytes;
        len -= nbytes;
        count += write(buffer, nbytes, dest);
    }
    return count;
}
size_t Logger::print(const String& s) {
    return print(LogDestination::Comms, s);
}

size_t Logger::print(LogDestination dest, const char s[]) {
    return write(s, dest);
}
size_t Logger::print(const char s[]) { return print(LogDestination::Comms, s); }

size_t Logger::print(LogDestination dest, char c) {
    return write((uint8_t)c, dest);
}
size_t Logger::print(char c) { return print(LogDestination::Comms, c); }

size_t Logger::print(LogDestination dest, unsigned char n, int base) {
    return printNumber(dest, n, base, 0);
}
size_t Logger::print(unsigned char n, int base) {
    return print(LogDestination::Comms, n, base);
}

size_t Logger::println(LogDestination dest) {
    uint8_t buf[2] = {'\r', '\n'};
    return write(buf, 2, dest);
}
size_t Logger::println() { return println(LogDestination::Comms); }

size_t Logger::println(LogDestination dest, const String& s) {
    return print(dest, s) + println(dest);
}
size_t Logger::println(const String& s) {
    return println(LogDestination::Comms, s);
}

size_t Logger::println(LogDestination dest, const char s[]) {
    return print(dest, s) + println(dest);
}
size_t Logger::println(const char s[]) {
    return println(LogDestination::Comms, s);
}

size_t Logger::println(LogDestination dest, char c) {
    return print(dest, c) + println(dest);
}
size_t Logger::println(char c) { return println(LogDestination::Comms, c); }

size_t Logger::println(LogDestination dest, unsigned char n, int base) {
    return print(dest, n, base) + println(dest);
}
size_t Logger::println(unsigned char n, int base) {
    return println(LogDestination::Comms, n, base);
}

// -------- print() unsigned numbers ------------------------------------------

size_t Logger::print(LogDestination dest, uint8_t b) {
    return printNumber(dest, b, 10, 0);
}
size_t Logger::print(uint8_t b) { return print(LogDestination::Comms, b); }

size_t Logger::print(LogDestination dest, unsigned int n) {
    return printNumber(dest, n, 10, 0);
}
size_t Logger::print(unsigned int n) { return print(LogDestination::Comms, n); }

size_t Logger::print(LogDestination dest, unsigned long n) {
    return printNumber(dest, n, 10, 0);
}
size_t Logger::print(unsigned long n) {
    return print(LogDestination::Comms, n);
}

size_t Logger::print(LogDestination dest, uint64_t n) {
    return printNumber64(dest, n, 10, 0);
}
size_t Logger::print(uint64_t n) { return print(LogDestination::Comms, n); }

size_t Logger::print(LogDestination dest, unsigned int n, int base) {
    return printNumber(dest, n, base, 0);
}
size_t Logger::print(unsigned int n, int base) {
    return print(LogDestination::Comms, n, base);
}

size_t Logger::print(LogDestination dest, unsigned long n, int base) {
    return printNumber(dest, n, base, 0);
}
size_t Logger::print(unsigned long n, int base) {
    return print(LogDestination::Comms, n, base);
}

size_t Logger::print(LogDestination dest, uint64_t n, int base) {
    return printNumber64(dest, n, base, 0);
}
size_t Logger::print(uint64_t n, int base) {
    return print(LogDestination::Comms, n, base);
}

size_t Logger::println(LogDestination dest, uint8_t b) {
    return print(dest, b) + println(dest);
}
size_t Logger::println(uint8_t b) { return println(LogDestination::Comms, b); }

size_t Logger::println(LogDestination dest, unsigned int n) {
    return print(dest, n) + println(dest);
}
size_t Logger::println(unsigned int n) {
    return println(LogDestination::Comms, n);
}

size_t Logger::println(LogDestination dest, unsigned long n) {
    return print(dest, n) + println(dest);
}
size_t Logger::println(unsigned long n) {
    return println(LogDestination::Comms, n);
}

size_t Logger::println(LogDestination dest, uint64_t n) {
    return print(dest, n) + println(dest);
}
size_t Logger::println(uint64_t n) { return println(LogDestination::Comms, n); }

size_t Logger::println(LogDestination dest, unsigned int n, int base) {
    return print(dest, n, base) + println(dest);
}
size_t Logger::println(unsigned int n, int base) {
    return println(LogDestination::Comms, n, base);
}

size_t Logger::println(LogDestination dest, unsigned long n, int base) {
    return print(dest, n, base) + println(dest);
}
size_t Logger::println(unsigned long n, int base) {
    return println(LogDestination::Comms, n, base);
}

size_t Logger::println(LogDestination dest, uint64_t n, int base) {
    return print(dest, n, base) + println(dest);
}
size_t Logger::println(uint64_t n, int base) {
    return println(LogDestination::Comms, n, base);
}

// -------- print() signed numbers --------------------------------------------

size_t Logger::print(LogDestination dest, int n) {
    return print(dest, (long)n);
}
size_t Logger::print(int n) { return print(LogDestination::Comms, n); }

size_t Logger::print(LogDestination dest, long n) {
    uint8_t sign = 0;
    if (n < 0) {
        sign = '-';
        n = -n;
    }
    return printNumber(dest, n, 10, sign);
}
size_t Logger::print(long n) { return print(LogDestination::Comms, n); }

size_t Logger::print(LogDestination dest, int64_t n) {
    if (n < 0) return printNumber64(dest, -n, 10, 1);
    return printNumber64(dest, n, 10, 0);
}
size_t Logger::print(int64_t n) { return print(LogDestination::Comms, n); }

size_t Logger::print(LogDestination dest, int n, int base) {
    return print(dest, (long)n, base);
}
size_t Logger::print(int n, int base) {
    return print(LogDestination::Comms, n, base);
}

size_t Logger::print(LogDestination dest, long n, int base) {
    return printNumber(dest, n, base, 0);
}
size_t Logger::print(long n, int base) {
    return print(LogDestination::Comms, n, base);
}

size_t Logger::print(LogDestination dest, int64_t n, int base) {
    if (n < 0) return printNumber64(dest, -n, base, 1);
    return printNumber64(dest, n, base, 0);
}
size_t Logger::print(int64_t n, int base) {
    return print(LogDestination::Comms, n, base);
}

size_t Logger::print(LogDestination dest, double n, int digits) {
    return printFloat(dest, n, digits);
}
size_t Logger::print(double n, int digits) {
    return print(LogDestination::Comms, n, digits);
}

size_t Logger::println(LogDestination dest, int n) {
    return print(dest, n) + println(dest);
}
size_t Logger::println(int n) { return println(LogDestination::Comms, n); }

size_t Logger::println(LogDestination dest, long n) {
    return print(dest, n) + println(dest);
}
size_t Logger::println(long n) { return println(LogDestination::Comms, n); }

size_t Logger::println(LogDestination dest, int64_t n) {
    return print(dest, n) + println(dest);
}
size_t Logger::println(int64_t n) { return println(LogDestination::Comms, n); }

size_t Logger::println(LogDestination dest, int n, int base) {
    return print(dest, n, base) + println(dest);
}
size_t Logger::println(int n, int base) {
    return println(LogDestination::Comms, n, base);
}

size_t Logger::println(LogDestination dest, long n, int base) {
    return print(dest, n, base) + println(dest);
}
size_t Logger::println(long n, int base) {
    return println(LogDestination::Comms, n, base);
}

size_t Logger::println(LogDestination dest, int64_t n, int base) {
    return print(dest, n, base) + println(dest);
}
size_t Logger::println(int64_t n, int base) {
    return println(LogDestination::Comms, n, base);
}

size_t Logger::println(LogDestination dest, double n, int digits) {
    return print(dest, n, digits) + println(dest);
}
size_t Logger::println(double n, int digits) {
    return println(LogDestination::Comms, n, digits);
}
