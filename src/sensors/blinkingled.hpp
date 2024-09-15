#include <FastLED.h>
#include <core_pins.h>
#include <iostream>

#define LED_PIN 6
#define NUM_LEDS 1 // Use fewer LEDs to maximize refresh rate
#define BRIGHTNESS 255

CRGB leds[NUM_LEDS];

void setup_LEDS()
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
}

void blinkLED()
{
    std::cout << "Entered blinkled function" << std::endl;

    setup_LEDS();
    // Toggle the LED on and off as fast as possible
    for (int i = 0; i < 100000; i++)
    {
        prof.begin("blink");
        leds[0] = CRGB::Black; // Turn off
        FastLED.show();
        delayMicroseconds(50); // Minimum reset time
        leds[0] = CRGB::White; // Turn on
        FastLED.show();
        delayMicroseconds(50); // inimum reset time
        prof.end("blink");
    }
    prof.print("blink");
}