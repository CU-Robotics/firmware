#include <Arduino.h>
#include <unity.h>

void setup();

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

extern "C" void unity_serial_putchar(int character) {
    Serial.write(static_cast<uint8_t>(character));
}

extern "C" void unity_serial_flush(void) {
    Serial.flush();
}

int main() {
    Serial.begin(115200);
    // The host starts the suite only after its monitor is ready to capture output.
    while (!Serial) { yield(); }
    while (Serial.read() != 'r') { yield(); }

    setup();
    Serial.printf("\nTEENSY_TEST_RESULT %u %u\n",
                  static_cast<unsigned>(Unity.NumberOfTests),
                  static_cast<unsigned>(Unity.TestFailures));
    Serial.flush();
    while (true) { yield(); }
}
