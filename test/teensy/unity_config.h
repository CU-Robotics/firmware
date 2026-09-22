#pragma once

#ifdef __cplusplus
extern "C" {
#endif
void unity_serial_putchar(int character);
void unity_serial_flush(void);
#ifdef __cplusplus
}
#endif

#define UNITY_OUTPUT_CHAR(character) unity_serial_putchar(character)
#define UNITY_OUTPUT_FLUSH() unity_serial_flush()
