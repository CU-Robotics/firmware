#include "logger.hpp"

#include "comms/comms_layer.hpp"
#include "comms/data/logging_data.hpp"
#include "comms/data/sendable.hpp"

/// @brief internal buffer with 4kb capacity
DMAMEM char log_buffer[LOGGER_BUFFER_SIZE];

// need to clear the buffer before using it
Logger::Logger() : cursor(0), write_error(0) {
    // Initialize the log buffer
    memset(log_buffer, 0, sizeof(log_buffer));
}

// -------- Logger internal functions -----------------------------------------

size_t Logger::write(const uint8_t *buffer, size_t size) {
    Serial.write(buffer, size);

    // guard against cursor exceeding buffer size
    if (cursor + size >= sizeof(log_buffer)) {
        Serial.println("!!! LOGGER BUFFER OVERFLOW !!!");
        return 0;
    }

    // writes incoming buffer to cursor position
    memcpy(log_buffer + cursor, buffer, size);
    cursor += size;
    return size;
}

size_t Logger::grab_log_data(size_t size, uint8_t *dest) {
    // ensure dest isn't a null pointer
    if (dest == nullptr) {
        return 0;
    }

    // make sure we don't copy more than the buffer size
    size_t bytes_to_copy = std::min(cursor, size);

    // copy internal buffer into *dest
    memcpy(dest, log_buffer, bytes_to_copy);

    // reset cursor
    cursor = 0;

    // return number of bytes copied
    return bytes_to_copy;
}

size_t Logger::printNumber(unsigned long n, uint8_t base, uint8_t sign) {
    uint8_t buf[34];
    uint8_t digit, i;

    // TODO: make these checks as inline, since base is
    // almost always a constant.  base = 0 (BYTE) should
    // inline as a call directly to write()
    if (base == 0) {
        return write((uint8_t)n);
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
            if (n == 0)
                break;
            i--;
        }
    }
    if (sign) {
        i--;
        buf[i] = '-';
    }
    return write(buf + i, sizeof(buf) - i);
}

size_t Logger::printNumber64(uint64_t n, uint8_t base, uint8_t sign) {
    uint8_t buf[66];
    uint8_t digit, i;

    if (base < 2)
        return 0;
    if (n == 0) {
        buf[sizeof(buf) - 1] = '0';
        i = sizeof(buf) - 1;
    } else {
        i = sizeof(buf) - 1;
        while (1) {
            digit = n % base;
            buf[i] = ((digit < 10) ? '0' + digit : 'A' + digit - 10);
            n /= base;
            if (n == 0)
                break;
            i--;
        }
    }
    if (sign) {
        i--;
        buf[i] = '-';
    }
    return write(buf + i, sizeof(buf) - i);
}

size_t Logger::printFloat(double number, uint8_t digits) {
    uint8_t sign = 0;
    size_t count = 0;

    if (isnan(number))
        return print("nan");
    if (isinf(number))
        return print("inf");
    if (number > 4294967040.0f)
        return print("ovf"); // constant determined empirically
    if (number < -4294967040.0f)
        return print("ovf"); // constant determined empirically

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
    count += printNumber(int_part, 10, sign);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        uint8_t n, buf[16], count = 1;
        buf[0] = '.';

        // Extract digits from the remainder one at a time
        if (digits > sizeof(buf) - 1)
            digits = sizeof(buf) - 1;

        while (digits-- > 0) {
            remainder *= 10.0;
            n = (uint8_t)(remainder);
            buf[count++] = '0' + n;
            remainder -= n;
        }
        count += write(buf, count);
    }
    return count;
}

// -------- print() strings and chars -----------------------------------------

size_t Logger::print(const String &s) {
    uint8_t buffer[33];
    size_t count = 0;
    unsigned int index = 0;
    unsigned int len = s.length();
    while (len > 0) {
        s.getBytes(buffer, sizeof(buffer), index);
        unsigned int nbytes = len;
        if (nbytes > sizeof(buffer) - 1)
            nbytes = sizeof(buffer) - 1;
        index += nbytes;
        len -= nbytes;
        count += write(buffer, nbytes);
    }
    return count;
}

size_t Logger::print(const char s[]) { return write(s); }
size_t Logger::print(char c) { return write((uint8_t)c); }

size_t Logger::print(unsigned char n, int base) { return printNumber(n, base, 0); }

size_t Logger::println() {
    uint8_t buf[2] = {'\r', '\n'};
    return write(buf, 2);
}

size_t Logger::println(const String &s) { return print(s) + println(); }
size_t Logger::println(const char s[]) { return print(s) + println(); }
size_t Logger::println(char c) { return print(c) + println(); }
size_t Logger::println(unsigned char n, int base) { return print(n, base) + println(); }

// -------- print() unsigned numbers ------------------------------------------

size_t Logger::print(uint8_t b) { return printNumber(b, 10, 0); }
size_t Logger::print(unsigned int n) { return printNumber(n, 10, 0); }
size_t Logger::print(unsigned long n) { return printNumber(n, 10, 0); }
size_t Logger::print(uint64_t n) { return printNumber64(n, 10, 0); }

size_t Logger::print(unsigned int n, int base) { return printNumber(n, base, 0); }
size_t Logger::print(unsigned long n, int base) { return printNumber(n, base, 0); }
size_t Logger::print(uint64_t n, int base) { return printNumber64(n, base, 0); }

size_t Logger::println(uint8_t b) { return print(b) + println(); }
size_t Logger::println(unsigned int n) { return print(n) + println(); }
size_t Logger::println(unsigned long n) { return print(n) + println(); }
size_t Logger::println(uint64_t n) { return print(n) + println(); }

size_t Logger::println(unsigned int n, int base) { return print(n, base) + println(); }
size_t Logger::println(unsigned long n, int base) { return print(n, base) + println(); }
size_t Logger::println(uint64_t n, int base) { return print(n, base) + println(); }

// -------- print() signed numbers --------------------------------------------

size_t Logger::print(int n) { return print((long)n); }

size_t Logger::print(long n) {
    uint8_t sign = 0;
    if (n < 0) {
        sign = '-';
        n = -n;
    }
    return printNumber(n, 10, sign);
}

size_t Logger::print(int64_t n) {
    if (n < 0)
        return printNumber64(-n, 10, 1);
    return printNumber64(n, 10, 0);
}

size_t Logger::print(int n, int base) { return print((long)n, base); }
size_t Logger::print(long n, int base) { return printNumber(n, base, 0); }

size_t Logger::print(int64_t n, int base) {
    if (n < 0)
        return printNumber64(-n, base, 1);
    return printNumber64(n, base, 0);
}

size_t Logger::print(double n, int digits) { return printFloat(n, digits); }

size_t Logger::println(int n) { return print(n) + println(); }
size_t Logger::println(long n) { return print(n) + println(); }
size_t Logger::println(int64_t n) { return print(n) + println(); }

size_t Logger::println(int n, int base) { return print(n, base) + println(); }
size_t Logger::println(long n, int base) { return print(n, base) + println(); }
size_t Logger::println(int64_t n, int base) { return print(n, base) + println(); }

size_t Logger::println(double n, int digits) { return print(n, digits) + println(); }

// -------- printf() ----------------------------------------------------------

// Function to handle the actual formatted printing
int Logger::vprintf(const char *format, va_list args) {
    uint8_t buffer[1024];
    int retval = vsnprintf_((char *)buffer, 1024, format, args);
    if (retval <= 0) {
        return retval;
    }

    size_t bytes_to_write = static_cast<size_t>(retval);
    if (bytes_to_write >= sizeof(buffer)) {
        bytes_to_write = sizeof(buffer) - 1;
    }

    write(buffer, bytes_to_write);

    return retval;
}

int Logger::printf(const char *format, ...) {
    va_list args;
    va_start(args, format);

    int retval = vprintf(format, args);

    va_end(args);

    return retval;
}
