//=============================================================================
// Default camera pins - we will define by Teensy type
// Note: These pins can be overrided in a sketch by first calling
// Set pins before the begin
//    void setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin,
//                 uint8_t hsync_pin, uint8_t en_pin,
//                 uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3,
//                 uint8_t g4 = 0xff, uint8_t g5 = 0xff,
//                 uint8_t g6 = 0xff, uint8_t g7 = 0xff,
//                 uint8_t shutdn_pin=0xff,
//                 TwoWire &wire = Wire);
//
//=============================================================================
#pragma once

#if defined(ARDUINO_TEENSY_DEVBRD4)
// BUGBUG this is not a real default config, but...
#define CAMERAPIN_XCLK 7
#define CAMERAPIN_PLK 8
#define CAMERAPIN_VSYNC 21
#define CAMERAPIN_HREF 46
#define CAMERAPIN_EN 17

// Note for FlexIO the D0-D7 need to be in sequence
#define CAMERAPIN_D0 40 // 40      B0_04   FlexIO2:4
#define CAMERAPIN_D1 41 // 41      B0_05   FlexIO2:5
#define CAMERAPIN_D2 42 // 42      B0_06   FlexIO2:6
#define CAMERAPIN_D3 43 // 43      B0_07   FlexIO2:7
#define CAMERAPIN_D4 44 // 44      B0_08   FlexIO2:8
#define CAMERAPIN_D5 45 // 45      B0_09   FlexIO2:9
#define CAMERAPIN_D6 6  // 6       B0_10   FlexIO2:10
#define CAMERAPIN_D7 9  // 9       B0_11   FlexIO2:11

#elif defined(ARDUINO_TEENSY_DEVBRD5)
// BUGBUG this is not a real default config, but...
#define CAMERAPIN_XCLK 7
#define CAMERAPIN_PLK 8
#define CAMERAPIN_VSYNC 21
#define CAMERAPIN_HREF 32
#define CAMERAPIN_EN 57

// Note for FlexIO the D0-D7 need to be in sequence
#define CAMERAPIN_D0 40 // 40      B0_04   FlexIO2:4
#define CAMERAPIN_D1 41 // 41      B0_05   FlexIO2:5
#define CAMERAPIN_D2 42 // 42      B0_06   FlexIO2:6
#define CAMERAPIN_D3 43 // 43      B0_07   FlexIO2:7
#define CAMERAPIN_D4 44 // 44      B0_08   FlexIO2:8
#define CAMERAPIN_D5 45 // 45      B0_09   FlexIO2:9
#define CAMERAPIN_D6 6  // 6       B0_10   FlexIO2:10
#define CAMERAPIN_D7 9  // 9       B0_11   FlexIO2:11


#elif defined(ARDUINO_TEENSY_MICROMOD)
// FLEXIO2 pins.
/*
HM01B0 pin      pin#    NXP     Usage
----------      ----    ---     -----
FVLD/VSYNC      33      EMC_07  GPIO
LVLD/HSYNC      32      B0_12   FlexIO2:12
MCLK            7       B1_01   PWM
PCLK            8       B1_00   FlexIO2:16
D0              40      B0_04   FlexIO2:4
D1              41      B0_05   FlexIO2:5
D2              42      B0_06   FlexIO2:6
D3              43      B0_07   FlexIO2:7
D4              44      B0_08   FlexIO2:8  - probably not needed, use 4 bit mode
D5              45      B0_09   FlexIO2:9  - probably not needed, use 4 bit mode
D6              6       B0_10   FlexIO2:10 - probably not needed, use 4 bit mode
D7              9       B0_11   FlexIO2:11 - probably not needed, use 4 bit mode
TRIG            5       EMC_08  ???
INT             29      EMC_31  ???
SCL             19      AD_B1_0 I2C
SDA             18      AD_B1_1 I2C
*/

#define CAMERAPIN_XCLK 29      // 7       B1_01   PWM
#define CAMERAPIN_PLK 10       // 8       B1_00   FlexIO2:16
#define CAMERAPIN_VSYNC 33     // 33 // 33      EMC_07  GPIO, 21 pon sdram board
#define CAMERAPIN_HREF 32      // 32      B0_12   FlexIO2:12, pin 46 on sdram board
#define CAMERAPIN_RST 31       // reset pin
#define CAMERAPIN_RST_INIT -2  // reset pin
#define CAMERAPIN_PWDN 30      // POWER down pin
#define CAMERAPIN_PWDN_INIT -1 // POWER down pin

#define CAMERAPIN_D0 40 // 40      B0_04   FlexIO2:4
#define CAMERAPIN_D1 41 // 41      B0_05   FlexIO2:5
#define CAMERAPIN_D2 42 // 42      B0_06   FlexIO2:6
#define CAMERAPIN_D3 43 // 43      B0_07   FlexIO2:7
#define CAMERAPIN_D4 44 // 44      B0_08   FlexIO2:8
#define CAMERAPIN_D5 45 // 45      B0_09   FlexIO2:9
#define CAMERAPIN_D6 6  // 6       B0_10   FlexIO2:10
#define CAMERAPIN_D7 9  // 9       B0_11   FlexIO2:11

#elif defined(ARDUINO_TEENSY41)

#define CAMERAPIN_XCLK 41  // AD_B1_05 CSI_MCLK
#define CAMERAPIN_PLK 40   // AD_B1_04 CSI_PIXCLK
#define CAMERAPIN_VSYNC 17 // AD_B1_06 CSI_VSYNC
#define CAMERAPIN_HREF 16  // AD_B1_07 CSI_HREF
#define CAMERAPIN_RST 14   // reset pin

#define CAMERAPIN_D0 27 // AD_B1_15 CSI_D2
#define CAMERAPIN_D1 26 // AD_B1_14 CSI_D3
#define CAMERAPIN_D2 39 // AD_B1_13 CSI_D4
#define CAMERAPIN_D3 38 // AD_B1_12 CSI_D5
#define CAMERAPIN_D4 21 // AD_B1_11 CSI_D6
#define CAMERAPIN_D5 20 // AD_B1_10 CSI_D7
#define CAMERAPIN_D6 23 // AD_B1_09 CSI_D8
#define CAMERAPIN_D7 22 // AD_B1_08 CSI_D9

#else
#define CAMERAPIN_PLK 4    // 40 // AD_B1_04 CSI_PIXCLK
#define CAMERAPIN_XCLK 5   // 41 // AD_B1_05 CSI_MCLK
#define CAMERAPIN_HREF 40  // AD_B1_07 CSI_HREF
#define CAMERAPIN_VSYNC 41 // AD_B1_06 CSI_VSYNC
#define CAMERAPIN_RST 0xff // don't know

#define CAMERAPIN_D0 27 // AD_B1_02 1.18
#define CAMERAPIN_D1 15 // AD_B1_03 1.19
#define CAMERAPIN_D2 17 // AD_B1_06 1.22
#define CAMERAPIN_D3 16 // AD_B1_07 1.23
#define CAMERAPIN_D4 22 // AD_B1_08 1.24
#define CAMERAPIN_D5 23 // AD_B1_09 1.25
#define CAMERAPIN_D6 20 // AD_B1_10 1.26
#define CAMERAPIN_D7 21 // AD_B1_11 1.27

#endif

//=============================================================================
// Defaults for optional pins
#ifndef CAMERAPIN_RST
#define CAMERAPIN_RST 0xff // reset pin
#endif

#ifndef CAMERAPIN_RST_INIT
#define CAMERAPIN_RST_INIT -1 // reset pin
#endif

#ifndef CAMERAPIN_PWDN
#define CAMERAPIN_PWDN 0xff // Power down pin
#endif

#ifndef CAMERAPIN_PWDN_INIT
#define CAMERAPIN_PWDN_INIT -1 // reset pin
#endif