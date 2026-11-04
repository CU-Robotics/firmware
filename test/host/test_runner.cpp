#include <unity.h>

void setup();

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

int main() {
    setup();
    return static_cast<int>(Unity.TestFailures);
}
