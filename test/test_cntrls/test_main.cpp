#include <Arduino.h>
#include <unity.h>

#include "controls/controller.hpp"

void setup() {
    delay(2000);

    UNITY_BEGIN();
    //RUN_TEST(test_wrap_around);
    //RUN_TEST(test_derivative);
    UNITY_END();
}


void loop() {}