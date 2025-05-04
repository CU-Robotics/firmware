#include <FastLED.h>
#include <core_pins.h>
#include "utils/profiler.hpp"
#include "LEDBoard.hpp"
 
// void ButtonLED::buttonLed_init()
// {
//     FastLED.addLeds<WS2812B, NEOPIXEL_PIN, GRB>(buttonBoard_led, num_leds);
//     FastLED.setBrightness(brightness);
// }

// // Update the LED based on the state of the robot
// void ButtonLED::buttonLed_status(uint16_t state)
// {
//     switch(state)
//     {
//         case(UNITIALIZED_STATE):
//             buttonBoard_led = CRGB::Red;
//             break;
//         case (SAFETY_MODE_STATE):
//             buttonBoard_led = CRGB::Green; 
//             break;
//     } 
// }

// Constructor
LEDBoard::LEDBoard(int num_leds, uint8_t brightness)
    : num_leds(num_leds), brightness(brightness) {
}

void LEDBoard::init() {
    setup_LEDS();
}


// Initialize the LEDs
void LEDBoard::setup_LEDS() {
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, num_leds).setRgbw(RgbwDefault());
    pinMode(33, OUTPUT);
    FastLED.setBrightness(brightness);
}

// Blink LEDs
void LEDBoard::blinkLED() {
    Serial.printf("in blinkled");
    // Toggle the LED on and off as fast as possible
    for (int i = 0; i < 10; i++) {
        prof.begin("blink");
        //leds[0] = CRGB::Red; // Turn off
        //leds[1] = CRGB::Red; // Turn off
        FastLED.show();
        prof.end("blink");
        delayMicroseconds(500000); // Minimum reset time
        leds[0] = CHSV(165,255, 255);   // Turn on
        //leds[1] = CRGB::Bla;
        FastLED.show();
        delayMicroseconds(500000); // Minimum reset time
    }
    leds[0] = CHSV(165,255, 255); // Turn off
    FastLED.show();
    prof.print("blink");
}

void LEDBoard::setState(state_t state){
    switch (state)
    {
    case STATE_UNINITIALIZED:
        /* code */
        leds[0] = CRGB::Magenta;
        FastLED.show();
        break;
    case STATE_INITIALIZED:
        leds[0] = CRGB::Green;
        FastLED.show();
        break;
    case STATE_CONFIGURED:
        leds[0] = CRGB::Blue;
        FastLED.show();
        break;
    case STATE_SAFETY_MODE_OFF:
        leds[0] = CRGB::Yellow;
        FastLED.show();
        break;
    case STATE_SAFETY_MODE_ON:
        leds[0] = CRGB::Green;
        FastLED.show();
        break;
    case STATE_ERROR:
        leds[0] = CRGB::Red;
        FastLED.show();
        delay(1000);
        leds[0] = CRGB::Black;
        FastLED.show();
        delay(1000);
        break;
    default:
        break;
    }

}

// Display a hexadecimal value on the LED matrix
void LEDBoard::displayHexOnMatrix(uint16_t hexValue) {
    // Go through the binary representation, 2 bits at a time
    for (int i = 0; i < num_leds; i++) {
        // Extract 2 bits
        uint8_t colorBits = (hexValue >> (2 * i)) & 0x03;

        // Map the 2 bits to a color
        leds[i] = getColorFromBits(colorBits);
    }
}

// Convert an integer to a hexadecimal value
uint16_t LEDBoard::convertIntegerToHex(int integerValue) {
    // Masking to fit into a 16-bit hex value (up to 0xFFFF)
    uint16_t hexValue = integerValue & 0xFFFF;
    return hexValue;
}

// Display an integer value on the LED matrix
void LEDBoard::displayIntOnMatrix(int integerValue) {
    uint16_t hexInput = convertIntegerToHex(integerValue);
    displayHexOnMatrix(hexInput);
}

// Set a specific LED on the matrix to a color
void LEDBoard::setLedOnMatrix(int led_index, CRGB color) {
    if (led_index >= 0 && led_index < num_leds) {
        leds[led_index] = color;
    }
    // Optionally handle invalid index
}

// Update the LED matrix display
void LEDBoard::updateLEDMatrix() {
    FastLED.show();
}



// Map 2 bits to a color
CRGB LEDBoard::getColorFromBits(uint8_t bits) {
    switch (bits) {
    case 0b00:
        return CRGB::Black; // Black
    case 0b01:
        return CRGB::Red; // Red
    case 0b10:
        return CRGB::Green; // Green
    case 0b11:
        return CRGB::Blue; // Blue
    default:
        return CRGB::Black; // Default to Black
    }
}
