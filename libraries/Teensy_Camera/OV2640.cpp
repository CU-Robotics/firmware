/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for
 * details.
 *
 * OV2640 driver.
 */

#include "OV2640.h"

#define debug Serial
// #define NO_CLK_PIN

// #define DEBUG_CAMERA
// #define DEBUG_CAMERA_VERBOSE
// #define DEBUG_FLEXIO
// #define USE_DEBUG_PINS
#define DEBUG_CAMERA_REG
#define USE_VSYNC_PIN_INT

// #define USE_DEBUG_PINS_TIMING

#ifdef USE_DEBUG_PINS_TIMING
#define DBGdigitalWriteFast digitalWriteFast
#define DBGdigitalToggleFast digitalToggleFast
#else
static inline void DBGdigitalWriteFast(uint8_t pin, uint8_t val)
    __attribute__((always_inline, unused));
static inline void DBGdigitalWriteFast(uint8_t pin, uint8_t val) {}
static inline void DBGdigitalToggleFast(uint8_t pin)
    __attribute__((always_inline, unused));
static inline void DBGdigitalToggleFast(uint8_t pin){}
#endif

// if not defined in the variant
#ifndef digitalPinToBitMask
#define digitalPinToBitMask(P) (1 << (digitalPinToPinName(P) % 64))
#endif

#ifndef portInputRegister
#define portInputRegister(P) ((P == 0) ? &NRF_P0->IN : &NRF_P1->IN)
#endif

#define CNT_SHIFTERS 1

#define CIF_WIDTH (400)
#define CIF_HEIGHT (296)

#define SVGA_WIDTH (800)
#define SVGA_HEIGHT (600)

#define UXGA_WIDTH (1600)
#define UXGA_HEIGHT (1200)

/** ln(10) */
#ifndef LN10
#define LN10 2.30258509299404568402f
#endif /* !M_LN10 */

/* log_e 2 */
#ifndef LN2
#define LN2 0.69314718055994530942
#endif /*!M_LN2 */

#define LOG2_2(x) (((x) & 0x2ULL) ? (2) : 1) // NO ({ ... }) !
#define LOG2_4(x) \
    (((x) & 0xCULL) ? (2 + LOG2_2((x) >> 2)) : LOG2_2(x)) // NO ({ ... }) !
#define LOG2_8(x) \
    (((x) & 0xF0ULL) ? (4 + LOG2_4((x) >> 4)) : LOG2_4(x)) // NO ({ ... }) !
#define LOG2_16(x) \
    (((x) & 0xFF00ULL) ? (8 + LOG2_8((x) >> 8)) : LOG2_8(x)) // NO ({ ... }) !
#define LOG2_32(x)                                     \
    (((x) & 0xFFFF0000ULL) ? (16 + LOG2_16((x) >> 16)) \
                           : LOG2_16(x)) // NO ({ ... }) !
#define LOG2(x)                                                \
    (((x) & 0xFFFFFFFF00000000ULL) ? (32 + LOG2_32((x) >> 32)) \
                                   : LOG2_32(x)) // NO ({ ... }) !

typedef struct {
    union {
        struct {
            uint8_t pclk_div : 7;
            uint8_t pclk_auto : 1;
        };
        uint8_t pclk;
    };
    union {
        struct {
            uint8_t clk_div : 6;
            uint8_t reserved : 1;
            uint8_t clk_2x : 1;
        };
        uint8_t clk;
    };
} ov2640_clk_t;

// Sensor frame size/resolution table.
const int resolution[][2] = {
    {640, 480},   /* VGA       */
    {160, 120},   /* QQVGA     */
    {320, 240},   /* QVGA      */
    {480, 320},   /* ILI9488   */
    {320, 320},   /* 320x320   */
    {320, 240},   /* QVGA      */
    {176, 144},   /* QCIF      */
    {352, 288},   /* CIF       */
    {800, 600},   /* SVGA      */
    {1600, 1200}, /* UXGA      */
    {0, 0},
};

static const uint8_t default_regs[][2] = {

    // From Linux Driver.

    {BANK_SEL, BANK_SEL_DSP},
    {0x2c, 0xff},
    {0x2e, 0xdf},
    {BANK_SEL, BANK_SEL_SENSOR},
    {0x3c, 0x32},
    {CLKRC, CLKRC_DOUBLE},
    {COM2, COM2_OUT_DRIVE_3x},
    {REG04, REG04_SET(REG04_HFLIP_IMG | REG04_VFLIP_IMG | REG04_VREF_EN |
                      REG04_HREF_EN)},
    {COM8, COM8_SET(COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN)},
    {COM9, COM9_AGC_SET(COM9_AGC_GAIN_8x)},
    {0x2c, 0x0c},
    {0x33, 0x78},
    {0x3a, 0x33},
    {0x3b, 0xfb},
    {0x3e, 0x00},
    {0x43, 0x11},
    {0x16, 0x10},
    {0x39, 0x02},
    {0x35, 0x88},
    {0x22, 0x0a},
    {0x37, 0x40},
    {0x23, 0x00},
    {ARCOM2, 0xa0},
    {0x06, 0x02},
    {0x06, 0x88},
    {0x07, 0xc0},
    {0x0d, 0xb7},
    {0x0e, 0x01},
    {0x4c, 0x00},
    {0x4a, 0x81},
    {0x21, 0x99},
    {AEW, 0x40},
    {AEB, 0x38},
    {VV, VV_AGC_TH_SET(0x08, 0x02)},
    {0x5c, 0x00},
    {0x63, 0x00},
    {FLL, 0x22},
    {COM3, COM3_BAND_SET(COM3_BAND_AUTO)},
    {REG5D, 0x55},
    {REG5E, 0x7d},
    {REG5F, 0x7d},
    {REG60, 0x55},
    {HISTO_LOW, 0x70},
    {HISTO_HIGH, 0x80},
    {0x7c, 0x05},
    {0x20, 0x80},
    {0x28, 0x30},
    {0x6c, 0x00},
    {0x6d, 0x80},
    {0x6e, 0x00},
    {0x70, 0x02},
    {0x71, 0x94},
    {0x73, 0xc1},
    {0x3d, 0x34},
    {COM7, COM7_RES_UXGA | COM7_ZOOM_EN},
    {0x5a, 0x57},
    {COM25, 0x00},
    {BD50, 0xbb},
    {BD60, 0x9c},
    {BANK_SEL, BANK_SEL_DSP},
    {0xe5, 0x7f},
    {MC_BIST, MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL},
    {0x41, 0x24},
    {RESET, RESET_JPEG | RESET_DVP},
    {0x76, 0xff},
    {0x33, 0xa0},
    {0x42, 0x20},
    {0x43, 0x18},
    {0x4c, 0x00},
    {CTRL3, CTRL3_BPC_EN | CTRL3_WPC_EN | 0x10},
    {0x88, 0x3f},
    {0xd7, 0x03},
    {0xd9, 0x10},
    {R_DVP_SP, R_DVP_SP_AUTO_MODE | 0x2},
    {0xc8, 0x08},
    {0xc9, 0x80},
    {BPADDR, 0x00},
    {BPDATA, 0x00},
    {BPADDR, 0x03},
    {BPDATA, 0x48},
    {BPDATA, 0x48},
    {BPADDR, 0x08},
    {BPDATA, 0x20},
    {BPDATA, 0x10},
    {BPDATA, 0x0e},
    {0x90, 0x00},
    {0x91, 0x0e},
    {0x91, 0x1a},
    {0x91, 0x31},
    {0x91, 0x5a},
    {0x91, 0x69},
    {0x91, 0x75},
    {0x91, 0x7e},
    {0x91, 0x88},
    {0x91, 0x8f},
    {0x91, 0x96},
    {0x91, 0xa3},
    {0x91, 0xaf},
    {0x91, 0xc4},
    {0x91, 0xd7},
    {0x91, 0xe8},
    {0x91, 0x20},
    {0x92, 0x00},
    {0x93, 0x06},
    {0x93, 0xe3},
    {0x93, 0x03},
    {0x93, 0x03},
    {0x93, 0x00},
    {0x93, 0x02},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x96, 0x00},
    {0x97, 0x08},
    {0x97, 0x19},
    {0x97, 0x02},
    {0x97, 0x0c},
    {0x97, 0x24},
    {0x97, 0x30},
    {0x97, 0x28},
    {0x97, 0x26},
    {0x97, 0x02},
    {0x97, 0x98},
    {0x97, 0x80},
    {0x97, 0x00},
    {0x97, 0x00},
    {0xa4, 0x00},
    {0xa8, 0x00},
    {0xc5, 0x11},
    {0xc6, 0x51},
    {0xbf, 0x80},
    {0xc7, 0x10}, /* simple AWB */
    {0xb6, 0x66},
    {0xb8, 0xA5},
    {0xb7, 0x64},
    {0xb9, 0x7C},
    {0xb3, 0xaf},
    {0xb4, 0x97},
    {0xb5, 0xFF},
    {0xb0, 0xC5},
    {0xb1, 0x94},
    {0xb2, 0x0f},
    {0xc4, 0x5c},
    {0xa6, 0x00},
    {0xa7, 0x20},
    {0xa7, 0xd8},
    {0xa7, 0x1b},
    {0xa7, 0x31},
    {0xa7, 0x00},
    {0xa7, 0x18},
    {0xa7, 0x20},
    {0xa7, 0xd8},
    {0xa7, 0x19},
    {0xa7, 0x31},
    {0xa7, 0x00},
    {0xa7, 0x18},
    {0xa7, 0x20},
    {0xa7, 0xd8},
    {0xa7, 0x19},
    {0xa7, 0x31},
    {0xa7, 0x00},
    {0xa7, 0x18},
    {0x7f, 0x00},
    {0xe5, 0x1f},
    {0xe1, 0x77},
    {0xdd, 0x7f},
    {CTRL0, CTRL0_YUV422 | CTRL0_YUV_EN | CTRL0_RGB_EN},

    // OpenMV Custom.

    {BANK_SEL, BANK_SEL_SENSOR},
    {0x0f, 0x4b},
    {COM1, 0x8f},

    // End.

    {0x00, 0x00},
};

// Looks really bad.
// static const uint8_t cif_regs[][2] = {
//    {BANK_SEL,  BANK_SEL_SENSOR},
//    {COM7,      COM7_RES_CIF},
//    {COM1,      0x06 | 0x80},
//    {HREFST,    0x11},
//    {HREFEND,     0x43},
//    {VSTRT,    0x01}, // 0x01 fixes issue with garbage pixels in the image...
//    {VEND,     0x97},
//    {REG32,     0x09},
//    {BANK_SEL,  BANK_SEL_DSP},
//    {RESET,     RESET_DVP},
//    {SIZEL,     SIZEL_HSIZE8_11_SET(CIF_WIDTH) | SIZEL_HSIZE8_SET(CIF_WIDTH) |
//    SIZEL_VSIZE8_SET(CIF_HEIGHT)}, {HSIZE8,    HSIZE8_SET(CIF_WIDTH)},
//    {VSIZE8,    VSIZE8_SET(CIF_HEIGHT)},
//    {CTRL2,     CTRL2_DCW_EN | CTRL2_SDE_EN | CTRL2_UV_AVG_EN | CTRL2_CMX_EN |
//    CTRL2_UV_ADJ_EN}, {0,         0},
//};

static const uint8_t svga_regs[][2] = {
    {BANK_SEL, BANK_SEL_SENSOR},
    {COM7, COM7_RES_SVGA},
    {COM1, 0x0A | 0x80},
    {HREFST, 0x11},
    {HREFEND, 0x43},
    {VSTRT, 0x01}, // 0x01 fixes issue with garbage pixels in the image...
    {VEND, 0x97},
    {REG32, 0x09},
    {BANK_SEL, BANK_SEL_DSP},
    {RESET, RESET_DVP},
    {SIZEL, SIZEL_HSIZE8_11_SET(SVGA_WIDTH) | SIZEL_HSIZE8_SET(SVGA_WIDTH) |
                SIZEL_VSIZE8_SET(SVGA_HEIGHT)},
    {HSIZE8, HSIZE8_SET(SVGA_WIDTH)},
    {VSIZE8, VSIZE8_SET(SVGA_HEIGHT)},
    {CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN | CTRL2_UV_AVG_EN | CTRL2_CMX_EN |
                CTRL2_UV_ADJ_EN},
    {0, 0},
};

static const uint8_t uxga_regs[][2] = {
    {BANK_SEL, BANK_SEL_SENSOR},
    {COM7, COM7_RES_UXGA},
    {COM1, 0x0F | 0x80},
    {HREFST, 0x11},
    {HREFEND, 0x75},
    {VSTRT, 0x01},
    {VEND, 0x97},
    {REG32, 0x36},
    {BANK_SEL, BANK_SEL_DSP},
    {RESET, RESET_DVP},
    {SIZEL, SIZEL_HSIZE8_11_SET(UXGA_WIDTH) | SIZEL_HSIZE8_SET(UXGA_WIDTH) |
                SIZEL_VSIZE8_SET(UXGA_HEIGHT)},
    {HSIZE8, HSIZE8_SET(UXGA_WIDTH)},
    {VSIZE8, VSIZE8_SET(UXGA_HEIGHT)},
    {CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN | CTRL2_UV_AVG_EN | CTRL2_CMX_EN |
                CTRL2_UV_ADJ_EN},
    {0, 0},
};

static const uint8_t yuv422_regs[][2] = {
    {BANK_SEL, BANK_SEL_DSP},
    {R_BYPASS, R_BYPASS_DSP_EN},
    {IMAGE_MODE, IMAGE_MODE_YUV422},
    {0xd7, 0x03},
    {0x33, 0xa0},
    {0xe5, 0x1f},
    {0xe1, 0x67},
    {RESET, 0x00},
    {R_BYPASS, R_BYPASS_DSP_EN},
    {0, 0},
};

static const uint8_t rgb565_regs[][2] = {
    {BANK_SEL, BANK_SEL_DSP},
    {R_BYPASS, R_BYPASS_DSP_EN},
    {IMAGE_MODE, IMAGE_MODE_RGB565},
    {0xd7, 0x03},
    {RESET, 0x00},
    {R_BYPASS, R_BYPASS_DSP_EN},
    {0, 0},
};

static const uint8_t bayer_regs[][2] = {
    {BANK_SEL, BANK_SEL_DSP},
    {R_BYPASS, R_BYPASS_DSP_EN},
    {IMAGE_MODE, IMAGE_MODE_RAW10},
    {0xd7, 0x03},
    {RESET, 0x00},
    {R_BYPASS, R_BYPASS_DSP_EN},
    {0, 0},
};

static const uint8_t jpeg_regs[][2] = {
    {BANK_SEL, BANK_SEL_DSP},
    {R_BYPASS, R_BYPASS_DSP_EN},
    {IMAGE_MODE, IMAGE_MODE_JPEG_EN},
    {0xd7, 0x03},
    {RESET, 0x00},
    {R_BYPASS, R_BYPASS_DSP_EN},
    {0, 0},
};

#define NUM_AE_LEVELS (5)
static const uint8_t ae_levels_regs[NUM_AE_LEVELS + 1][3] = {
    {AEW, AEB, VV},
    {0x20, 0X18, 0x60},
    {0x34, 0X1C, 0x00},
    {0x3E, 0X38, 0x81},
    {0x48, 0X40, 0x81},
    {0x58, 0X50, 0x92},
};

#define NUM_BRIGHTNESS_LEVELS (5)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS + 1][5] = {
    {BPADDR, BPDATA, BPADDR, BPDATA, BPDATA},
    {0x00, 0x04, 0x09, 0x00, 0x00}, /* -2 */
    {0x00, 0x04, 0x09, 0x10, 0x00}, /* -1 */
    {0x00, 0x04, 0x09, 0x20, 0x00}, /*  0 */
    {0x00, 0x04, 0x09, 0x30, 0x00}, /* +1 */
    {0x00, 0x04, 0x09, 0x40, 0x00}, /* +2 */
};

#define NUM_CONTRAST_LEVELS (5)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS + 1][7] = {
    {BPADDR, BPDATA, BPADDR, BPDATA, BPDATA, BPDATA, BPDATA},
    {0x00, 0x04, 0x07, 0x20, 0x18, 0x34, 0x06}, /* -2 */
    {0x00, 0x04, 0x07, 0x20, 0x1c, 0x2a, 0x06}, /* -1 */
    {0x00, 0x04, 0x07, 0x20, 0x20, 0x20, 0x06}, /*  0 */
    {0x00, 0x04, 0x07, 0x20, 0x24, 0x16, 0x06}, /* +1 */
    {0x00, 0x04, 0x07, 0x20, 0x28, 0x0c, 0x06}, /* +2 */
};

#define NUM_SATURATION_LEVELS (5)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS + 1][5] = {
    {BPADDR, BPDATA, BPADDR, BPDATA, BPDATA},
    {0x00, 0x02, 0x03, 0x28, 0x28}, /* -2 */
    {0x00, 0x02, 0x03, 0x38, 0x38}, /* -1 */
    {0x00, 0x02, 0x03, 0x48, 0x48}, /*  0 */
    {0x00, 0x02, 0x03, 0x58, 0x58}, /* +1 */
    {0x00, 0x02, 0x03, 0x68, 0x68}, /* +2 */
};

#define NUM_SPECIAL_EFFECTS (7)
static const uint8_t special_effects_regs[NUM_SPECIAL_EFFECTS + 1][5] = {
    {BPADDR, BPDATA, BPADDR, BPDATA, BPDATA},
    {0x00, 0X00, 0x05, 0X80, 0X80}, /* no effect */
    {0x00, 0X40, 0x05, 0X80, 0X80}, /* negative */
    {0x00, 0X18, 0x05, 0X80, 0X80}, /* black and white */
    {0x00, 0X18, 0x05, 0X40, 0XC0}, /* reddish */
    {0x00, 0X18, 0x05, 0X40, 0X40}, /* greenish */
    {0x00, 0X18, 0x05, 0XA0, 0X40}, /* blue */
    {0x00, 0X18, 0x05, 0X40, 0XA6}, /* retro */
};

#define NUM_WB_MODES (4)
static const uint8_t wb_modes_regs[NUM_WB_MODES + 1][3] = {
    {0XCC, 0XCD, 0XCE}, {0x5E, 0X41, 0x54}, /* sunny */
    {0x65, 0X41, 0x4F},                     /* cloudy */
    {0x52, 0X41, 0x66},                     /* office */
    {0x42, 0X3F, 0x71},                     /* home */
};

const uint8_t agc_gain_tbl[31] = {
    0x00, 0x10, 0x18, 0x30, 0x34, 0x38, 0x3C, 0x70, 0x72, 0x74, 0x76,
    0x78, 0x7A, 0x7C, 0x7E, 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6,
    0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF};

OV2640::OV2640()
    : _OV2640(NULL), _saturation(3), _hue(0), _frame_buffer_pointer(NULL) {
}

// Read a single uint8_t from address and return it as a uint8_t
uint8_t OV2640::cameraReadRegister(uint8_t reg) {
    _wire->beginTransmission(0x30);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
        debug.println("error reading OV2640, address");
        return 0;
    }
    if (_wire->requestFrom(0x30, 1) < 1) {
        debug.println("error reading OV2640, data");
        return 0;
    }
    return _wire->read();
}

uint8_t OV2640::cameraWriteRegister(uint8_t reg, uint8_t data) {
#ifdef DEBUG_CAMERA_REG
    extern void Debug_printCameraWriteRegister(uint8_t reg, uint8_t data);
    if (_debug)
        Debug_printCameraWriteRegister(reg, data);
#endif
    _wire->beginTransmission(0x30);
    _wire->write(reg);
    _wire->write(data);
    if (_wire->endTransmission() != 0) {
        debug.println("error writing to OV2640");
    }
    return 0;
}

uint16_t OV2640::getModelid() {
    uint8_t Data;
    uint16_t MID = 0x0000;

    Data = cameraReadRegister(0x0A);
    MID = (Data << 8);

    Data = cameraReadRegister(0x0B);
    MID |= Data;

    if (_debug)
        debug.printf("getModelID: return: %x\n", MID);
    return MID;
}

// int OV2640::begin(int resolution, int format, int fps,  int camera_name, bool
// use_gpio)
bool OV2640::begin_omnivision(framesize_t framesize, pixformat_t format,
                              int fps, int camera_name, bool use_gpio) {

    _use_gpio = use_gpio;

    // WIP - Need set functions:
    if (_rst != 0xff) {
        if (_rst_init >= 0) {
            pinMode(_rst, OUTPUT);
            digitalWrite(_rst, _rst_init);
        } else if (_rst_init == -1)
            pinMode(_rst, INPUT);
        else if (_rst_init == -2)
            pinMode(_rst, INPUT_PULLUP);
        else if (_rst_init == -3)
            pinMode(_rst, INPUT_PULLDOWN);
        delay(5);
    }

    if (_pwdn != 0xff) {
        if (_pwdn_init >= 0) {
            pinMode(_pwdn, OUTPUT);
            digitalWrite(_pwdn, _pwdn_init);
        } else if (_pwdn_init == -1)
            pinMode(_pwdn, INPUT);
        else if (_pwdn_init == -2)
            pinMode(_pwdn, INPUT_PULLUP);
        else if (_pwdn_init == -3)
            pinMode(_pwdn, INPUT_PULLDOWN);
        delay(5);
    }

// BUGBUG::: see where frame is
#ifdef USE_DEBUG_PINS
    pinMode(49, OUTPUT);
#endif

    //_wire = &Wire;
    _wire->begin();

    if (framesize >= (sizeof(resolution) / sizeof(resolution[0])))
        return 1; // error

    _width = resolution[framesize][0];
    if (_width == 0) {
        if (_debug)
            debug.println("Frame Size Invalid!!!");
        return false;
    }
    _height = resolution[framesize][1];
    _framesize = (uint8_t)framesize;

    _grayscale = false;
    switch (format) {
    case YUV422:
        _bytesPerPixel = 2;
        _format = 0;
        break;
    case BAYER:
        _bytesPerPixel = 2;
        _format = 1;
        break;
    case RGB565:
        _bytesPerPixel = 2;
        _format = 2;
        break;

    case GRAYSCALE:
        format = YUV422;    // We use YUV422 but discard U and V bytes
        _bytesPerPixel = 2; // 2 input bytes per pixel of which 1 is discarded
        _grayscale = true;
        _format = 4;
        break;
    case JPEG:
        _bytesPerPixel = 2;
        _format = 8;
        break;
    default:
        return false;
    }

    pinMode(_vsyncPin, INPUT /*INPUT_PULLDOWN*/);
    //  const struct digital_pin_bitband_and_config_table_struct *p;
    //  p = digital_pin_to_info_PGM + _vsyncPin;
    //  *(p->pad) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_HYS;  // See if I turn on
    //  HYS...
    pinMode(_hrefPin, INPUT_PULLUP);
    pinMode(_pclkPin, INPUT_PULLDOWN);
    pinMode(_xclkPin, OUTPUT);

#ifdef DEBUG_CAMERA
    debug.printf("  VS=%d, HR=%d, PC=%d XC=%d\n", _vsyncPin, _hrefPin, _pclkPin,
                 _xclkPin);
    debug.printf("  RST=%d(%d), PWDN=%d(%d)\n", _rst, _rst_init, _pwdn, _pwdn_init);

    for (int i = 0; i < 8; i++) {
        pinMode(_dPins[i], INPUT);
        debug.printf("  _dpins(%d)=%d\n", i, _dPins[i]);
    }
#endif

    _vsyncPort = portInputRegister(digitalPinToPort(_vsyncPin));
    _vsyncMask = digitalPinToBitMask(_vsyncPin);
    _hrefPort = portInputRegister(digitalPinToPort(_hrefPin));
    _hrefMask = digitalPinToBitMask(_hrefPin);
    _pclkPort = portInputRegister(digitalPinToPort(_pclkPin));
    _pclkMask = digitalPinToBitMask(_pclkPin);

    /*
      if(camera_name == OV7670) {
          _xclk_freq = 14;  //was 16Mhz
      } else {
          if(fps <= 10){
           _xclk_freq = 14;
          } else {
          _xclk_freq = 16;
          }
      }
    */

    beginXClk();

    if (_rst != 0xFF) {
        pinMode(_rst, OUTPUT);
        digitalWriteFast(_rst, LOW); /* Reset */
        for (volatile uint32_t i = 0; i < 100000; i++) {
        }
        digitalWriteFast(_rst, HIGH); /* Normal mode. */
        for (volatile uint32_t i = 0; i < 100000; i++) {
        }
    }

    _wire->begin();

    delay(1000);

    if (getModelid() != 0x2641 && getModelid() != 0x2642) {
        end();
        if (_debug)
            debug.println("Camera detect failed");
        return false;
    }

#ifdef DEBUG_CAMERA
    debug.printf("Calling ov7670_configure\n");
    debug.printf("Cam Name: %d, Format: %d, Resolution: %d, Clock: %d\n",
                 camera_name, _format, _framesize, _xclk_freq);
    debug.printf("Frame rate: %d\n", fps);
#endif

    // flexIO/DMA
    if (!_use_gpio) {
        hardware_configure();
        setVSyncISRPriority(102);
        setDMACompleteISRPriority(192);
    } else {
        setVSyncISRPriority(102);
        setDMACompleteISRPriority(192);
    }

    reset();
    if (setPixformat(format) != 0) {
        if (_debug)
            debug.println("Error: setPixformat failed");
        return false;
    }
    if (setFramesize(framesize) != 0) {
        if (_debug)
            debug.println("Error: setFramesize failed");
        return false; // failed to set resolution
    }

    // for now frame rate is fixed

    return true;
}

int OV2640::reset() {
    int ret = 0;

    for (int i = 0; default_regs[i][0] && ret == 0; i++) {
        ret |= cameraWriteRegister(default_regs[i][0], default_regs[i][1]);
    }

    // Delay 10 ms
    delay(10);

    return ret;
}

void OV2640::end() {
    endXClk();

    pinMode(_xclkPin, INPUT);

    _wire->end();
}

int OV2640::setPixformat(pixformat_t pixformat) {
    // const uint8_t(*regs)[2];
    int ret = 0;

    switch (pixformat) {
    case RGB565:
        _format = 5;
        // regs = rgb565_regs;
        for (int i = 0; rgb565_regs[i][0] && ret == 0; i++) {
            ret |= cameraWriteRegister(rgb565_regs[i][0], rgb565_regs[i][1]);
        }
        break;
    case YUV422:
    case GRAYSCALE:
        _format = 3;
        // regs = yuv422_regs;
        for (int i = 0; yuv422_regs[i][0] && ret == 0; i++) {
            ret |= cameraWriteRegister(yuv422_regs[i][0], yuv422_regs[i][1]);
        }
        break;
    case BAYER:
        _format = 5;
        // regs = bayer_regs;
        for (int i = 0; bayer_regs[i][0] && ret == 0; i++) {
            ret |= cameraWriteRegister(bayer_regs[i][0], bayer_regs[i][1]);
        }
        break;
    case JPEG:
        _format = 8;
        // regs = jpeg_regs;
        for (int i = 0; jpeg_regs[i][0] && ret == 0; i++) {
            ret |= cameraWriteRegister(jpeg_regs[i][0], jpeg_regs[i][1]);
        }
        break;
    default:
        return 0;
    }

    return ret;
}

uint8_t OV2640::setFramesize(framesize_t framesize) {
    if (framesize >= (sizeof(resolution) / sizeof(resolution[0])))
        return 1; // error
    return setFramesize(resolution[framesize][0], resolution[framesize][1]);
}

uint8_t OV2640::setFramesize(int w, int h) {
    uint16_t sensor_w = 0;
    uint16_t sensor_h = 0;
    int ret = 0;

    ov2640_clk_t c;

    if ((w == 0) || (h == 0))
        return 1; // not valid

    // uint16_t w = resolution[framesize][0];
    // uint16_t h = resolution[framesize][1];

    if (_debug) {
        debug.printf("UXGA Width: %d, Height: %d\n", UXGA_WIDTH, UXGA_HEIGHT);
        debug.printf("Width: %d, Height: %d\n", w, h);
    }

    if ((w % 4) || (h % 4) || (w > UXGA_WIDTH) || (h > UXGA_HEIGHT)) {
        // w/h must be divisible by 4
        if (_debug) {
            debug.println("width/height must be divisible by 4");
        }
        return 0;
    }

    // Looks really bad.
    /* if ((w <= CIF_WIDTH) && (h <= CIF_HEIGHT)) {
        regs = cif_regs;
        sensor_w = CIF_WIDTH;
        sensor_h = CIF_HEIGHT;
       } else */
    if ((w <= SVGA_WIDTH) && (h <= SVGA_HEIGHT)) {
        // regs = svga_regs;
        for (int i = 0; svga_regs[i][0] && ret == 0; i++) {
            ret |= cameraWriteRegister(svga_regs[i][0], svga_regs[i][1]);
        }
        sensor_w = SVGA_WIDTH;
        sensor_h = SVGA_HEIGHT;
    } else {
        // regs = uxga_regs;
        for (int i = 0; uxga_regs[i][0] && ret == 0; i++) {
            ret |= cameraWriteRegister(uxga_regs[i][0], uxga_regs[i][1]);
        }
        sensor_w = UXGA_WIDTH;
        sensor_h = UXGA_HEIGHT;
    }

    _width = w;
    _height = h;
    _frame_width = sensor_w;
    _frame_height = sensor_h;

    uint64_t tmp_div = min(sensor_w / w, sensor_h / h);
    uint16_t log_div = min(LOG2(tmp_div) - 1, 3);
    uint16_t div = 1 << log_div;
    uint16_t w_mul = w * div;
    uint16_t h_mul = h * div;
    uint16_t x_off = (sensor_w - w_mul) / 2;
    uint16_t y_off = (sensor_h - h_mul) / 2;

    if (_debug) {
        debug.printf("Sensor w,h: %d - %d\n", sensor_w, sensor_h);
        debug.printf("sensor_w / w: %d, sensor_h / h: %d\n", sensor_w / w,
                     sensor_h / h);
        debug.printf("temp_div: %d, log_div: %d, div: %d\n", tmp_div, log_div,
                     div);
        debug.printf("w_mul: %d, h_mul: %d\n", w_mul, h_mul);
        debug.printf("x_off: %d, y_off: %d\n", x_off, y_off);
    }

    ret |= cameraWriteRegister(CTRLI, CTRLI_LP_DP | CTRLI_V_DIV_SET(log_div) |
                                          CTRLI_H_DIV_SET(log_div));
    ret |= cameraWriteRegister(HSIZE, HSIZE_SET(w_mul));
    ret |= cameraWriteRegister(VSIZE, VSIZE_SET(h_mul));
    ret |= cameraWriteRegister(XOFFL, XOFFL_SET(x_off));
    ret |= cameraWriteRegister(YOFFL, YOFFL_SET(y_off));
    ret |= cameraWriteRegister(VHYX,
                               VHYX_HSIZE_SET(w_mul) | VHYX_VSIZE_SET(h_mul) |
                                   VHYX_XOFF_SET(x_off) | VHYX_YOFF_SET(y_off));
    ret |= cameraWriteRegister(TEST, TEST_HSIZE_SET(w_mul));
    ret |= cameraWriteRegister(ZMOW, ZMOW_OUTW_SET(w));
    ret |= cameraWriteRegister(ZMOH, ZMOH_OUTH_SET(h));
    ret |= cameraWriteRegister(ZMHH, ZMHH_OUTW_SET(w) | ZMHH_OUTH_SET(h));
    ret |= cameraWriteRegister(R_DVP_SP, div);
    ret |= cameraWriteRegister(RESET, 0x00);

    // Extracted from the ESP32 Camera implementation to address cameras
    // without CLK Pins
    if (_format == 8) { // jpeg
        c.clk_2x = 0;
        c.clk_div = 0;
        c.pclk_auto = 0;
        c.pclk_div = 8;
        if (w <= SVGA_WIDTH) {
            c.pclk_div = 12;
        }
        // if (sensor->xclk_freq_hz == 16000000) {
        //     c.pclk_div = c.pclk_div / 2;
        // }
    } else {
        if (_use_gpio) {
            c.clk_2x = 0;
        } else {
            c.clk_2x = 1; // ELSE 1
        }
        c.clk_div = 7;
        c.pclk_auto = 1;
        c.pclk_div = 8;
        if (w <= CIF_WIDTH) {
            if (_use_gpio) {
                c.clk_div = 8;
            } else {
                c.clk_div = 3;
            }
        } else if (w <= SVGA_WIDTH) {
            c.pclk_div = 12;
        }
    }

    if (_debug) {
        debug.printf(
            "Set PLL: clk_2x: %u, clk_div: %u, pclk_auto: %u, pclk_div: %u\n",
            c.clk_2x, c.clk_div, c.pclk_auto, c.pclk_div);
        debug.printf("c.clk value: %u\n", c.clk);
        debug.printf("c.pclk value: %u\n", c.pclk);
    }

    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);
    ret |= cameraWriteRegister(CLKRC, c.clk);
    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
    ret |= cameraWriteRegister(R_DVP_SP, c.pclk);
    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
    ret |= cameraWriteRegister(R_BYPASS, R_BYPASS_DSP_EN);

    delay(10);

    return ret;
}

bool OV2640::setZoomWindow(uint16_t x_off, uint16_t y_off, uint16_t w,
                           uint16_t h) {
    if (_debug)
        debug.printf("OV2640::setZoomWindow(%u %u %u %u)\n", x_off, y_off, w,
                     h);

    if (w == (uint16_t)-1)
        w = _width;
    if (h == (uint16_t)-1)
        h = _height;
    if ((w > _frame_width) || (h > _frame_height))
        return false;

    if (x_off == (uint16_t)-1)
        x_off = (_frame_width - w) / 2;
    if (y_off == (uint16_t)-1)
        y_off = (_frame_height - h) / 2;

    if ((x_off + w) > _frame_width)
        return false;
    if ((y_off + h) > _frame_height)
        return false;

    uint64_t tmp_div = min(_frame_width / w, _frame_height / h);
    uint16_t log_div = min(LOG2(tmp_div) - 1, 3);
    uint16_t div = 1 << log_div;
    uint16_t w_mul = w * div;
    uint16_t h_mul = h * div;

    // Set up to use the display bank sel
    cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
    bool width_or_height_changed = (w != _width) || (h != _height);
    if (width_or_height_changed) {
        if ((w % 4) || (h % 4)) {
            // w/h must be divisible by 4
            if (_debug) {
                debug.println("width/height must be divisible by 4");
            }
            return false;
        }
        cameraWriteRegister(CTRLI, CTRLI_LP_DP | CTRLI_V_DIV_SET(log_div) |
                                       CTRLI_H_DIV_SET(log_div));
        cameraWriteRegister(HSIZE, HSIZE_SET(w_mul));
        cameraWriteRegister(VSIZE, VSIZE_SET(h_mul));
    }
    cameraWriteRegister(XOFFL, XOFFL_SET(x_off));
    cameraWriteRegister(YOFFL, YOFFL_SET(y_off));

    cameraWriteRegister(VHYX, VHYX_HSIZE_SET(w_mul) | VHYX_VSIZE_SET(h_mul) |
                                  VHYX_XOFF_SET(x_off) | VHYX_YOFF_SET(y_off));
    cameraWriteRegister(TEST, TEST_HSIZE_SET(w_mul));
    cameraWriteRegister(ZMOW, ZMOW_OUTW_SET(w));
    cameraWriteRegister(ZMOH, ZMOH_OUTH_SET(h));
    cameraWriteRegister(ZMHH, ZMHH_OUTW_SET(w) | ZMHH_OUTH_SET(h));
    cameraWriteRegister(R_DVP_SP, div);
    cameraWriteRegister(RESET, 0x00);

    if (_debug) {
        debug.printf("Frame w,h: %d - %d\n", _frame_width, _frame_height);
        debug.printf("zoom w, h: %u %u\n", w, h);
        debug.printf("x_off: %u(%u), y_off: %u(%u)\n", x_off, XOFFL_SET(x_off),
                     y_off, YOFFL_SET(y_off));
    }
    _width = w;
    _height = h;

    if (0) { //(width_or_height_changed) {
        ov2640_clk_t c;
        if (_format == 8) { // jpeg
            c.clk_2x = 0;
            c.clk_div = 0;
            c.pclk_auto = 0;
            c.pclk_div = 8;
            if (w <= SVGA_WIDTH) {
                c.pclk_div = 12;
            }
            // if (sensor->xclk_freq_hz == 16000000) {
            //     c.pclk_div = c.pclk_div / 2;
            // }
        } else {
#if defined(NO_CLK_PIN)
            c.clk_2x = 0;
#else
            c.clk_2x = 1; // ELSE 1
#endif
            c.clk_div = 7;
            c.pclk_auto = 1;
            c.pclk_div = 8;
            if (w <= CIF_WIDTH) {
#if defined(NO_CLK_PIN)
                c.clk_div = 8;
#else
                c.clk_div = 3;
#endif
            } else if (w <= SVGA_WIDTH) {
                c.pclk_div = 12;
            }
        }

        if (_debug) {
            debug.printf("Set PLL: clk_2x: %u, clk_div: %u, pclk_auto: %u, "
                         "pclk_div: %u\n",
                         c.clk_2x, c.clk_div, c.pclk_auto, c.pclk_div);
            debug.printf("c.clk value: %u\n", c.clk);
            debug.printf("c.pclk value: %u\n", c.pclk);
        }

        cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);
        cameraWriteRegister(CLKRC, c.clk);
        cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
        cameraWriteRegister(R_DVP_SP, c.pclk);
        cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
        cameraWriteRegister(R_BYPASS, R_BYPASS_DSP_EN);

        delay(10);
    }

    return true;
}

void OV2640::setContrast(int level) {
    int ret = 0;

    level += (NUM_CONTRAST_LEVELS / 2) + 1;
    if (level <= 0 || level > NUM_CONTRAST_LEVELS) {
        if (_debug)
            debug.println("ERROR: Contrast Levels Exceeded !!!");
        level = 5;
    }

    /* Switch to DSP register bank */
    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);

    /* Write contrast registers */
    for (int i = 0; i < NUM_CONTRAST_LEVELS; i++) {
        ret |=
            cameraWriteRegister(contrast_regs[0][i], contrast_regs[level][i]);
    }

    // return ret;
}

int OV2640::setBrightness(int level) {
    int ret = 0;

    level += (NUM_BRIGHTNESS_LEVELS / 2) + 1;
    if (level <= 0 || level > NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }

    /* Switch to DSP register bank */
    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);

    /* Write brightness registers */
    for (int i = 0; i < NUM_BRIGHTNESS_LEVELS; i++) {
        ret |= cameraWriteRegister(brightness_regs[0][i],
                                   brightness_regs[level][i]);
    }

    return ret;
}

void OV2640::setSaturation(int level) {
    int ret = 0;

    level += (NUM_SATURATION_LEVELS / 2) + 1;
    if (level <= 0 || level > NUM_SATURATION_LEVELS) {
        // return 0;
        if (_debug)
            debug.println("ERROR: Saturation levels exceeded!!");
        level = 5;
    }

    /* Switch to DSP register bank */
    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);

    /* Write saturation registers */
    for (int i = 0; i < NUM_SATURATION_LEVELS; i++) {
        ret |= cameraWriteRegister(saturation_regs[0][i],
                                   saturation_regs[level][i]);
    }

    // return ret;
}

int OV2640::setGainceiling(gainceiling_t gainceiling) {
    int ret = 0;

    /* Switch to SENSOR register bank */
    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);

    /* Write gain ceiling register */
    ret |= cameraWriteRegister(COM9, COM9_AGC_SET(gainceiling));

    return ret;
}

int OV2640::setQuality(int qs) {
    int ret = 0;

    /* Switch to DSP register bank */
    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);

    /* Write QS register */
    ret |= cameraWriteRegister(QS, qs);

    return ret;
}

uint8_t OV2640::getQuality() {
    // int ret = 0;

    /* Switch to DSP register bank */
    cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);

    /* Write QS register */
    return cameraReadRegister(QS);
}

int OV2640::setColorbar(int enable) {
    uint8_t reg;
    int ret = cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);
    reg = cameraReadRegister(COM7);

    if (enable) {
        reg |= COM7_COLOR_BAR;
    } else {
        reg &= ~COM7_COLOR_BAR;
    }
    return cameraWriteRegister(COM7, reg) | ret;
}

void OV2640::autoExposure(
    int enable) // enable is really exposure level for the 2640
{
    enable += 3;
    if (enable <= 0 || enable > NUM_AE_LEVELS) {
        debug.println("ERROR: Auto exposure level out of range!!!");
    } else {
        enable = enable - 3;
        for (int i = 0; i < 3; i++) {
            cameraWriteRegister(ae_levels_regs[0][i],
                                ae_levels_regs[enable][i]);
        }
    }
}

void OV2640::setGain(int gain) {
    if (gain < 0) {
        gain = 0;
    } else if (gain > 30) {
        gain = 30;
    }
    cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);
    cameraWriteRegister(GAIN, agc_gain_tbl[gain]);
}

int OV2640::setAutoGain(int enable, float gain_db, float gain_db_ceiling) {
    uint8_t reg;
    int ret = cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);
    reg = cameraReadRegister(COM8);
    ret |= cameraWriteRegister(COM8, (reg & (~COM8_AGC_EN)) |
                                         ((enable != 0) ? COM8_AGC_EN : 0));

    if ((enable == 0) && (!isnanf(gain_db)) && (!isinff(gain_db))) {
        float gain = max(min(expf((gain_db / 20.0f) * LN10), 32.0f), 1.0f);

        int gain_temp = fast_ceilf(logf(max(gain / 2.0f, 1.0f)) / M_LN2);
        int gain_hi = 0xF >> (4 - gain_temp);
        int gain_lo =
            min(fast_roundf(((gain / (1 << gain_temp)) - 1.0f) * 16.0f), 15);

        ret |= cameraWriteRegister(GAIN, (gain_hi << 4) | (gain_lo << 0));
    } else if ((enable != 0) && (!isnanf(gain_db_ceiling)) &&
               (!isinff(gain_db_ceiling))) {
        float gain_ceiling =
            max(min(expf((gain_db_ceiling / 20.0f) * LN10), 128.0f), 2.0f);

        reg = cameraReadRegister(COM9);
        ret |= cameraWriteRegister(
            COM9,
            (reg & 0x1F) | ((fast_ceilf(logf(gain_ceiling) / M_LN2) - 1) << 5));
    }

    return ret;
}

int OV2640::getGain_db(float *gain_db) {
    // uint8_t reg;
    uint8_t gain;
    int ret = cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);

    // Currently disabled
    // reg = cameraReadRegister( COM8 );

    // DISABLED
    // if (reg & COM8_AGC_EN) {
    //     ret |= cameraWriteRegister(  COM8, reg & (~COM8_AGC_EN));
    // }
    // DISABLED

    gain = cameraReadRegister(GAIN);

    // DISABLED
    // if (reg & COM8_AGC_EN) {
    //     ret |= cameraWriteRegister(  COM8, reg | COM8_AGC_EN);
    // }
    // DISABLED

    int hi_gain = 1 << (((gain >> 7) & 1) + ((gain >> 6) & 1) +
                        ((gain >> 5) & 1) + ((gain >> 4) & 1));
    float lo_gain = 1.0f + (((gain >> 0) & 0xF) / 16.0f);
    *gain_db = 20.0f * log10f(hi_gain * lo_gain);

    return ret;
}

int OV2640::setAutoExposure(int enable, int exposure_us) {
    uint8_t reg;
    int ret = cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);
    reg = cameraReadRegister(COM8);
    ret |= cameraWriteRegister(COM8, COM8_SET_AEC(reg, (enable != 0)));

    if ((enable == 0) && (exposure_us >= 0)) {
        reg = cameraReadRegister(COM7);
        int t_line = 0;

        if (COM7_GET_RES(reg) == COM7_RES_UXGA) {
            t_line = 1600 + 322;
        }
        if (COM7_GET_RES(reg) == COM7_RES_SVGA) {
            t_line = 800 + 390;
        }
        if (COM7_GET_RES(reg) == COM7_RES_CIF) {
            t_line = 400 + 195;
        }

        reg = cameraReadRegister(CLKRC);
        int pll_mult = ((reg & CLKRC_DOUBLE) ? 2 : 1) * 3;
        int clk_rc = (reg & CLKRC_DIVIDER_MASK) + 2;

        ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
        reg = cameraReadRegister(IMAGE_MODE);
        int t_pclk = 0;

        if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_YUV422) {
            t_pclk = 2;
        }
        if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_RAW10) {
            t_pclk = 1;
        }
        if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_RGB565) {
            t_pclk = 2;
        }

        int exposure = max(
            min(((exposure_us *
                  ((((_xclk_freq * 1000000) / clk_rc) * pll_mult) / 1000000)) /
                 t_pclk) /
                    t_line,
                0xFFFF),
            0x0000);

        ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);

        reg = cameraReadRegister(REG04);
        ret |=
            cameraWriteRegister(REG04, (reg & 0xFC) | ((exposure >> 0) & 0x3));

        reg = cameraReadRegister(AEC);
        ret |=
            cameraWriteRegister(AEC, (reg & 0x00) | ((exposure >> 2) & 0xFF));

        reg = cameraReadRegister(REG45);
        ret |= cameraWriteRegister(REG45,
                                   (reg & 0xC0) | ((exposure >> 10) & 0x3F));
    }

    return ret;
}

int OV2640::getExposure_us(int *exposure_us) {
    uint8_t reg, aec_10, aec_92, aec_1510;
    int ret = cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);
    reg = cameraReadRegister(COM8);

    // DISABLED
    // if (reg & COM8_AEC_EN) {
    //     ret |= cameraWriteRegister(  COM8, reg & (~COM8_AEC_EN));
    // }
    // DISABLED

    aec_10 = cameraReadRegister(REG04);
    aec_92 = cameraReadRegister(AEC);
    aec_1510 = cameraReadRegister(REG45);

    // DISABLED
    // if (reg & COM8_AEC_EN) {
    //     ret |= cameraWriteRegister(  COM8, reg | COM8_AEC_EN);
    // }
    // DISABLED

    reg = cameraReadRegister(COM7);
    int t_line = 0;

    if (COM7_GET_RES(reg) == COM7_RES_UXGA) {
        t_line = 1600 + 322;
    }
    if (COM7_GET_RES(reg) == COM7_RES_SVGA) {
        t_line = 800 + 390;
    }
    if (COM7_GET_RES(reg) == COM7_RES_CIF) {
        t_line = 400 + 195;
    }

    reg = cameraReadRegister(CLKRC);
    int pll_mult = ((reg & CLKRC_DOUBLE) ? 2 : 1) * 3;
    int clk_rc = (reg & CLKRC_DIVIDER_MASK) + 2;

    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
    reg = cameraReadRegister(IMAGE_MODE);
    int t_pclk = 0;

    if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_YUV422) {
        t_pclk = 2;
    }
    if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_RAW10) {
        t_pclk = 1;
    }
    if (IMAGE_MODE_GET_FMT(reg) == IMAGE_MODE_RGB565) {
        t_pclk = 2;
    }

    uint16_t exposure = ((aec_1510 & 0x3F) << 10) + ((aec_92 & 0xFF) << 2) +
                        ((aec_10 & 0x3) << 0);
    *exposure_us = (exposure * t_line * t_pclk) /
                   ((((_xclk_freq * 1000000) / clk_rc) * pll_mult) / 1000000);

    return ret;
}

int OV2640::setAutoWhitebal(int enable, float r_gain_db, float g_gain_db,
                            float b_gain_db) {
    uint8_t reg;
    int ret = cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
    reg = cameraReadRegister(CTRL1);
    ret |= cameraWriteRegister(CTRL1, (reg & (~CTRL1_AWB)) |
                                          ((enable != 0) ? CTRL1_AWB : 0));

    if ((enable == 0) && (!isnanf(r_gain_db)) && (!isnanf(g_gain_db)) &&
        (!isnanf(b_gain_db)) && (!isinff(r_gain_db)) && (!isinff(g_gain_db)) &&
        (!isinff(b_gain_db))) {
    }

    return ret;
}

int OV2640::getRGB_Gain_db(float *r_gain_db, float *g_gain_db,
                           float *b_gain_db) {
    // uint8_t reg;
    int ret = cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
    // reg = cameraReadRegister(  CTRL1 );

    // DISABLED
    // if (reg & CTRL1_AWB) {
    //     ret |= cameraWriteRegister(  CTRL1, reg & (~CTRL1_AWB));
    // }
    // DISABLED

    // DISABLED
    // if (reg & CTRL1_AWB) {
    //     ret |= cameraWriteRegister(  CTRL1, reg | CTRL1_AWB);
    // }
    // DISABLED

    *r_gain_db = NAN;
    *g_gain_db = NAN;
    *b_gain_db = NAN;

    return ret;
}

int OV2640::setHmirror(int enable) {
    uint8_t reg;
    int ret = cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);
    reg = cameraReadRegister(REG04);

    if (!enable) {
        // Already mirrored.
        reg |= REG04_HFLIP_IMG;
    } else {
        reg &= ~REG04_HFLIP_IMG;
    }

    return cameraWriteRegister(REG04, reg) | ret;
}

int OV2640::setVflip(int enable) {
    uint8_t reg;
    int ret = cameraWriteRegister(BANK_SEL, BANK_SEL_SENSOR);
    reg = cameraReadRegister(REG04);

    if (!enable) {
        // Already flipped.
        reg |= REG04_VFLIP_IMG | REG04_VREF_EN;
    } else {
        reg &= ~(REG04_VFLIP_IMG | REG04_VREF_EN);
    }

    return cameraWriteRegister(REG04, reg) | ret;
}

int OV2640::setSpecialEffect(sde_t sde) {
    int ret = 0;
    int effect;

    effect = sde;
    effect++;
    if (effect <= 0 || effect > NUM_SPECIAL_EFFECTS) {
        return 1;
    }

    /* Switch to DSP register bank */
    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);

    /* Write special effect registers */
    for (int i = 0; i < 5; i++) {
        ret |= cameraWriteRegister(special_effects_regs[0][i],
                                   special_effects_regs[effect][i]);
    }

    return ret;
}

int OV2640::setWBmode(int mode) {
    int ret = 0;
    if (mode < 0 || mode > NUM_WB_MODES) {
        /* Switch to DSP register bank */
        ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);
        /* AWB ON */
        ret |= cameraWriteRegister(0xC7, 0x00);
        return 1;
    }

    /* Switch to DSP register bank */
    ret |= cameraWriteRegister(BANK_SEL, BANK_SEL_DSP);

    /* AWB OFF */
    ret |= cameraWriteRegister(0xC7, 0x40);

    if (mode) {
        for (int i = 0; i < 3; i++) {
            ret |= cameraWriteRegister(wb_modes_regs[0][i],
                                       wb_modes_regs[mode][i]);
        }
    }
    return ret;
}

/*******************************************************************/

#define FLEXIO_USE_DMA

size_t OV2640::readFrameGPIO_JPEG(void *buffer, size_t cb1, void *buffer2,
                                  size_t cb2) {

    uint16_t w = _width;
    uint16_t h = _height;
    uint32_t i_count = 0;

    debug.printf("$$readFrameGPIO(%p, %u, %p, %u)\n", buffer, cb1, buffer2,
                 cb2);
    const uint32_t frame_size_bytes = w * h * _bytesPerPixel / 5;
    if ((cb1 + cb2) < frame_size_bytes) {
        if (_debug)
            debug.printf("Warning Buffers may be too small %u < %u\n",
                         cb1 + cb2, frame_size_bytes);
    }
    DBGdigitalWriteFast(0, HIGH);
    uint8_t *b = (uint8_t *)buffer;
    uint32_t cb = (uint32_t)cb1;
    //  bool _grayscale;  // ????  member variable ?????????????
    int bytesPerRow = _width * _bytesPerPixel;

    // Falling edge indicates start of frame
    // pinMode(PCLK_PIN, INPUT); // make sure back to input pin...
    // lets add our own glitch filter.  Say it must be hig for at least 100us
    elapsedMicros emHigh;
    DBGdigitalWriteFast(0, LOW);
    do {
        while ((*_vsyncPort & _vsyncMask) == 0)
            ; // wait for HIGH
        emHigh = 0;
        while ((*_vsyncPort & _vsyncMask) != 0)
            ; // wait for LOW
    } while (emHigh < 1);

    // uint8_t *pu8 = (uint8_t *)b;
    uint8_t prev_char = 0;

    DBGdigitalWriteFast(0, HIGH);
    for (int i = 0; i < h; i++) {
        // rising edge indicates start of line
        while ((*_hrefPort & _hrefMask) == 0)
            ; // wait for HIGH
        while ((*_pclkPort & _pclkMask) != 0)
            ; // wait for LOW
        noInterrupts();

        for (int j = 0; j < bytesPerRow; j++) {
            // rising edges clock each data byte
            while ((*_pclkPort & _pclkMask) == 0)
                ; // wait for HIGH

            // uint32_t in = ((_frame_buffer_pointer)? GPIO1_DR : GPIO6_DR) >>
            // 18; // read all bits in parallel
            uint32_t in = (GPIO7_PSR >> 4); // read all bits in parallel

            // uint32_t in = mmBus;
            // bugbug what happens to the the data if grayscale?
            if (!(j & 1) || !_grayscale) {
                *b++ = in;
                cb--;
                if (cb == 0) {
                    if (buffer2) {
                        if (_debug)
                            debug.printf("\t$$ 2nd buffer: %u %u\n", i, j);
                        b = (uint8_t *)buffer2;
                        cb = (uint32_t)cb2;
                        buffer2 = nullptr;
                    } else {
                        if (_debug)
                            debug.printf("Error failed buffers too small\n");
                        interrupts();
                        DBGdigitalWriteFast(0, LOW);
                        return frame_size_bytes;
                    }
                }
            }

            if ((prev_char == 0xff) && ((uint8_t)in == 0xd9)) {
                interrupts();
                DBGdigitalWriteFast(0, LOW);
                return i_count + 1;
            }
            prev_char = (uint8_t)in;
            i_count = i_count + 1;

            while (((*_pclkPort & _pclkMask) != 0) &&
                   ((*_hrefPort & _hrefMask) != 0))
                ; // wait for LOW bail if _href is lost
        }

        while ((*_hrefPort & _hrefMask) != 0)
            ; // wait for LOW
        interrupts();
    }
    DBGdigitalWriteFast(0, LOW);

    return frame_size_bytes;
}

/*********************************************************************/

//======================================== DMA JUNK
//================================================================================
// experiment with DMA
//================================================================================
// Define our DMA structure.
DMAChannel OV2640::_dmachannel;
DMASetting OV2640::_dmasettings[10];
uint32_t OV2640::_dmaBuffer1[DMABUFFER_SIZE] __attribute__((used, aligned(32)));
uint32_t OV2640::_dmaBuffer2[DMABUFFER_SIZE] __attribute__((used, aligned(32)));
extern "C" void xbar_connect(unsigned int input,
                             unsigned int output); // in pwm.c

// OV2640 *OV2640::active_dma_camera = nullptr;

//===================================================================
// Start a DMA operation -
//===================================================================
#if 0 // def later
bool OV2640::startReadFrameDMA(bool(*callback)(void *frame_buffer), uint8_t *fb1, uint8_t *fb2) {return false;}
bool OV2640::stopReadFrameDMA() {return false;}

#else
bool OV2640::startReadFrameDMA(bool (*callback)(void *frame_buffer),
                               uint8_t *fb1, uint8_t *fb2) {
    // First see if we need to allocate frame buffers.
    if (fb1)
        _frame_buffer_1 = fb1;
    else if (_frame_buffer_1 == nullptr) {
        _frame_buffer_1 = (uint8_t *)malloc(_width * _height);
        if (_frame_buffer_1 == nullptr)
            return false;
    }
    if (fb2)
        _frame_buffer_2 = fb2;
    else if (_frame_buffer_2 == nullptr) {
        _frame_buffer_2 = (uint8_t *)malloc(_width * _height);
        if (_frame_buffer_2 == nullptr)
            return false; // BUGBUG should we 32 byte align?
    }
    // remember the call back if passed in
    _callback = callback;
    active_dma_camera = this;

    if (_debug)
        debug.printf("startReadFrameDMA called buffers %x %x\n",
                     (uint32_t)_frame_buffer_1, (uint32_t)_frame_buffer_2);

    // DebugDigitalToggle(OV7670_DEBUG_PIN_1);
    // lets figure out how many bytes we will tranfer per setting...
    //  _dmasettings[0].begin();
    _frame_row_buffer_pointer = _frame_buffer_pointer =
        (uint8_t *)_frame_buffer_1;

    // configure DMA channels
    _dmachannel.begin();
    _dmasettings[0].source(GPIO2_PSR); // setup source.
    _dmasettings[0].destinationBuffer(
        _dmaBuffer1, DMABUFFER_SIZE * 4); // 32 bits per logical byte
    _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);
    _dmasettings[0]
        .interruptAtCompletion(); // we will need an interrupt to process this.
    _dmasettings[0].TCD->CSR &=
        ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
    // DebugDigitalToggle(OV7670_DEBUG_PIN_1);

    _dmasettings[1].source(GPIO2_PSR); // setup source.
    _dmasettings[1].destinationBuffer(
        _dmaBuffer2, DMABUFFER_SIZE * 4); // 32 bits per logical byte
    _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
    _dmasettings[1]
        .interruptAtCompletion(); // we will need an interrupt to process this.
    _dmasettings[1].TCD->CSR &=
        ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
    // DebugDigitalToggle(OV7670_DEBUG_PIN_1);

    GPIO2_GDIR = 0; // set all as input...
    GPIO2_DR = 0;   // see if I can clear it out...

    _dmachannel = _dmasettings[0]; // setup the first on...
    _dmachannel.attachInterrupt(dmaInterrupt);
    _dmachannel.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);
    // DebugDigitalToggle(OV7670_DEBUG_PIN_1);

    // Lets try to setup the DMA setup...
    // first see if we can convert the _pclk to be an XBAR Input pin...
    // OV7670_PLK   4
    // OV7670_PLK   8    //8       B1_00   FlexIO2:16  XBAR IO14

    _save_pclkPin_portConfigRegister = *(portConfigRegister(_pclkPin));
    *(portConfigRegister(_pclkPin)) = 1; // set to XBAR mode 14

    // route the timer outputs through XBAR to edge trigger DMA request
    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
    xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT14, XBARA1_OUT_DMA_CH_MUX_REQ30);
    // DebugDigitalToggle(OV7670_DEBUG_PIN_1);

    // Tell XBAR to dDMA on Rising
    XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(1) |
                   XBARA_CTRL_DEN0 /* | XBARA_CTRL_IEN0 */;

    IOMUXC_GPR_GPR6 &=
        ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_14); // Make sure it is input mode
    IOMUXC_XBAR1_IN14_SELECT_INPUT =
        1; // Make sure this signal goes to this pin...

#if defined(ARDUINO_TEENSY_MICROMOD)
    // Need to switch the IO pins back to GPI1 from GPIO6
    _save_IOMUXC_GPR_GPR27 =
        IOMUXC_GPR_GPR27; // save away the configuration before we change...
    IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

    // lets also un map the _hrefPin to GPIO1
    IOMUXC_GPR_GPR27 &= ~_hrefMask; //
#else
    // Need to switch the IO pins back to GPI1 from GPIO6
    _save_IOMUXC_GPR_GPR26 =
        IOMUXC_GPR_GPR26; // save away the configuration before we change...
    IOMUXC_GPR_GPR26 &= ~(0x0ff0u);

    // lets also un map the _hrefPin to GPIO1
    IOMUXC_GPR_GPR26 &= ~_hrefMask; //
#endif

    // Need to switch the IO pins back to GPI1 from GPIO6
    //_save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration
    // before we change... IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

    // lets also un map the _hrefPin to GPIO1
    // IOMUXC_GPR_GPR27 &= ~_hrefMask; //

    // DebugDigitalToggle(OV7670_DEBUG_PIN_1);

    // Falling edge indicates start of frame
    //  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
    //  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
    //  DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);

    // Debug stuff for now

    // We have the start of a frame, so lets start the dma.
#ifdef DEBUG_CAMERA
    dumpDMA_TCD(&_dmachannel, " CH: ");
    dumpDMA_TCD(&_dmasettings[0], " 0: ");
    dumpDMA_TCD(&_dmasettings[1], " 1: ");

    debug.printf("pclk pin: %d config:%lx control:%lx\n", _pclkPin,
                 *(portConfigRegister(_pclkPin)),
                 *(portControlRegister(_pclkPin)));
    debug.printf("IOMUXC_GPR_GPR26-29:%lx %lx %lx %lx\n", IOMUXC_GPR_GPR26,
                 IOMUXC_GPR_GPR27, IOMUXC_GPR_GPR28, IOMUXC_GPR_GPR29);
    debug.printf("GPIO1: %lx %lx, GPIO6: %lx %lx\n", GPIO1_DR, GPIO1_PSR,
                 GPIO6_DR, GPIO6_PSR);
    debug.printf("XBAR CTRL0:%x CTRL1:%x\n\n", XBARA1_CTRL0, XBARA1_CTRL1);
#endif
    _dma_state = DMASTATE_RUNNING;
    _dma_last_completed_frame = nullptr;
    _dma_frame_count = 0;

    // Now start an interrupt for start of frame.
    //  attachInterrupt(_vsyncPin, &frameStartInterrupt, RISING);

    // DebugDigitalToggle(OV7670_DEBUG_PIN_1);
    return true;
}

//===================================================================
// stopReadFrameDMA - stop doing the reading and then exit.
//===================================================================
bool OV2640::stopReadFrameDMA() {

// hopefully it start here (fingers crossed)
// for now will hang here to see if completes...
#ifdef OV7670_USE_DEBUG_PINS
// DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);
#endif
    elapsedMillis em = 0;
    // tell the background stuff DMA stuff to exit.
    // Note: for now let it end on on, later could disable the DMA directly.
    _dma_state = DMASTATE_STOP_REQUESTED;

    while ((em < 1000) && (_dma_state == DMASTATE_STOP_REQUESTED))
        ; // wait up to a second...
    if (_dma_state != DMA_STATE_STOPPED) {
        debug.println("*** stopReadFrameDMA DMA did not exit correctly...");
        debug.printf("  Bytes Left: %u frame buffer:%x Row:%u Col:%u\n",
                     _bytes_left_dma, (uint32_t)_frame_buffer_pointer,
                     _frame_row_index, _frame_col_index);
    }
#ifdef OV7670_USE_DEBUG_PINS
// DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);
#endif
#ifdef DEBUG_CAMERA
    dumpDMA_TCD(&_dmachannel, nullptr);
    dumpDMA_TCD(&_dmasettings[0], nullptr);
    dumpDMA_TCD(&_dmasettings[1], nullptr);
    debug.println();
#endif
    // Lets restore some hardware pieces back to the way we found them.
#if defined(ARDUINO_TEENSY_MICROMOD)
    IOMUXC_GPR_GPR27 =
        _save_IOMUXC_GPR_GPR27; // Restore... away the configuration before we
                                // change...
#else
    IOMUXC_GPR_GPR26 =
        _save_IOMUXC_GPR_GPR26; // Restore... away the configuration before we
                                // change...
#endif
    *(portConfigRegister(_pclkPin)) = _save_pclkPin_portConfigRegister;

    return (em < 1000); // did we stop...
}

//===================================================================
// Our Frame Start interrupt.
//===================================================================
#if 0
void  OV2640::frameStartInterrupt() {
  active_dma_camera->processFrameStartInterrupt();  // lets get back to the main object...
}

void  OV2640::processFrameStartInterrupt() {
  _bytes_left_dma = (_width + _frame_ignore_cols) * _height; // for now assuming color 565 image...
  _dma_index = 0;
  _frame_col_index = 0;  // which column we are in a row
  _frame_row_index = 0;  // which row
  _save_lsb = 0xffff;
  // make sure our DMA is setup properly again. 
  _dmasettings[0].transferCount(DMABUFFER_SIZE);
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmasettings[1].transferCount(DMABUFFER_SIZE);
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.enable();
  
  detachInterrupt(_vsyncPin);
}
#endif

//===================================================================
// Our DMA interrupt.
//===================================================================
void OV2640::dmaInterrupt() {
    active_dma_camera
        ->processDMAInterrupt(); // lets get back to the main object...
}

// This version assumes only called when HREF...  as set pixclk to only fire
// when set.
void OV2640::processDMAInterrupt() {
    _dmachannel.clearInterrupt(); // tell system we processed it.
    asm("DSB");
#ifdef USE_DEBUG_PINS
// DebugDigitalWrite(OV7670_DEBUG_PIN_3, HIGH);
#endif

    if (_dma_state == DMA_STATE_STOPPED) {
        debug.println("OV2640::dmaInterrupt called when DMA_STATE_STOPPED");
        return; //
    }

    // lets guess which buffer completed.
    uint32_t *buffer;
    uint16_t buffer_size;
    _dma_index++;
    if (_dma_index & 1) {
        buffer = _dmaBuffer1;
        buffer_size = _dmasettings[0].TCD->CITER;

    } else {
        buffer = _dmaBuffer2;
        buffer_size = _dmasettings[1].TCD->CITER;
    }
    // lets try dumping a little data on 1st 2nd and last buffer.
#ifdef DEBUG_CAMERA_VERBOSE
    if ((_dma_index < 3) || (buffer_size < DMABUFFER_SIZE)) {
        debug.printf("D(%d, %d, %lu) %u : ", _dma_index, buffer_size,
                     _bytes_left_dma, pixformat);
        for (uint16_t i = 0; i < 8; i++) {
            uint16_t b = buffer[i] >> 4;
            debug.printf(" %lx(%02x)", buffer[i], b);
        }
        debug.print("...");
        for (uint16_t i = buffer_size - 8; i < buffer_size; i++) {
            uint16_t b = buffer[i] >> 4;
            debug.printf(" %lx(%02x)", buffer[i], b);
        }
        debug.println();
    }
#endif

    for (uint16_t buffer_index = 0; buffer_index < buffer_size;
         buffer_index++) {
        if (!_bytes_left_dma || (_frame_row_index >= _height))
            break;

        // only process if href high...
        uint16_t b = *buffer >> 4;
        *_frame_buffer_pointer++ = b;
        _frame_col_index++;
        if (_frame_col_index == _width) {
            // we just finished a row.
            _frame_row_index++;
            _frame_col_index = 0;
        }
        _bytes_left_dma--; // for now assuming color 565 image...
        buffer++;
    }

    if ((_frame_row_index == _height) ||
        (_bytes_left_dma == 0)) { // We finished a frame lets bail
        _dmachannel.disable();    // disable the DMA now...
#ifdef USE_DEBUG_PINS
// DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);
#endif
#ifdef DEBUG_CAMERA_VERBOSE
        debug.println("EOF");
#endif
        _frame_row_index = 0;
        _dma_frame_count++;

        bool swap_buffers = true;

        // DebugDigitalToggle(OV7670_DEBUG_PIN_1);
        _dma_last_completed_frame = _frame_row_buffer_pointer;
        if (_callback)
            swap_buffers = (*_callback)(_dma_last_completed_frame);

        if (swap_buffers) {
            if (_frame_row_buffer_pointer != _frame_buffer_1)
                _frame_row_buffer_pointer = _frame_buffer_2;
            else
                _frame_row_buffer_pointer = _frame_buffer_2;
        }

        _frame_buffer_pointer = _frame_row_buffer_pointer;

        // DebugDigitalToggle(OV7670_DEBUG_PIN_1);

        if (_dma_state == DMASTATE_STOP_REQUESTED) {
#ifdef DEBUG_CAMERA
            debug.println("OV2640::dmaInterrupt - Stop requested");
#endif
            _dma_state = DMA_STATE_STOPPED;
        } else {
            // We need to start up our ISR for the next frame.
#if 1
            // bypass interrupt and just restart DMA...
            _bytes_left_dma = (_width + _frame_ignore_cols) *
                              _height; // for now assuming color 565 image...
            _dma_index = 0;
            _frame_col_index = 0; // which column we are in a row
            _frame_row_index = 0; // which row
            _save_lsb = 0xffff;
            // make sure our DMA is setup properly again.
            _dmasettings[0].transferCount(DMABUFFER_SIZE);
            _dmasettings[0].TCD->CSR &=
                ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
            _dmasettings[1].transferCount(DMABUFFER_SIZE);
            _dmasettings[1].TCD->CSR &=
                ~(DMA_TCD_CSR_DREQ);       // Don't disable on this one
            _dmachannel = _dmasettings[0]; // setup the first on...
            _dmachannel.enable();

#else
            attachInterrupt(_vsyncPin, &frameStartInterrupt, RISING);
#endif
        }
    } else {

        if (_bytes_left_dma == (2 * DMABUFFER_SIZE)) {
            if (_dma_index & 1)
                _dmasettings[0].disableOnCompletion();
            else
                _dmasettings[1].disableOnCompletion();
        }
    }
#ifdef OV7670_USE_DEBUG_PINS
// DebugDigitalWrite(OV7670_DEBUG_PIN_3, LOW);
#endif
}

#endif // LATER
typedef struct {
    uint32_t frameTimeMicros;
    uint16_t vsyncStartCycleCount;
    uint16_t vsyncEndCycleCount;
    uint16_t hrefCount;
    uint32_t cycleCount;
    uint16_t pclkCounts[350]; // room to spare.
    uint32_t hrefStartTime[350];
    uint16_t pclkNoHrefCount;
} frameStatics_t;

frameStatics_t fstatOmni1;

void OV2640::captureFrameStatistics() {
    memset((void *)&fstatOmni1, 0, sizeof(fstatOmni1));

    // lets wait for the vsync to go high;
    while ((*_vsyncPort & _vsyncMask) != 0)
        ; // wait for HIGH
    // now lets wait for it to go low
    while ((*_vsyncPort & _vsyncMask) == 0)
        fstatOmni1.vsyncStartCycleCount++; // wait for LOW

    while ((*_hrefPort & _hrefMask) == 0)
        ; // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0)
        ; // wait for LOW

    uint32_t microsStart = micros();
    fstatOmni1.hrefStartTime[0] = microsStart;
    // now loop through until we get the next _vsynd
    // BUGBUG We know that HSYNC and PCLK on same GPIO VSYNC is not...
    uint32_t regs_prev = 0;
    // noInterrupts();
    while ((*_vsyncPort & _vsyncMask) != 0) {

        fstatOmni1.cycleCount++;
        uint32_t regs = (*_hrefPort & (_hrefMask | _pclkMask));
        if (regs != regs_prev) {
            if ((regs & _hrefMask) && ((regs_prev & _hrefMask) == 0)) {
                fstatOmni1.hrefCount++;
                fstatOmni1.hrefStartTime[fstatOmni1.hrefCount] = micros();
            }
            if ((regs & _pclkMask) && ((regs_prev & _pclkMask) == 0))
                fstatOmni1.pclkCounts[fstatOmni1.hrefCount]++;
            if ((regs & _pclkMask) && ((regs_prev & _hrefMask) == 0))
                fstatOmni1.pclkNoHrefCount++;
            regs_prev = regs;
        }
    }
    while ((*_vsyncPort & _vsyncMask) == 0)
        fstatOmni1.vsyncEndCycleCount++; // wait for LOW
    // interrupts();
    fstatOmni1.frameTimeMicros = micros() - microsStart;

    // Maybe return data. print now
    debug.printf("*** Frame Capture Data: elapsed Micros: %u loops: %u\n",
                 fstatOmni1.frameTimeMicros, fstatOmni1.cycleCount);
    debug.printf("   VSync Loops Start: %u end: %u\n",
                 fstatOmni1.vsyncStartCycleCount,
                 fstatOmni1.vsyncEndCycleCount);
    debug.printf("   href count: %u pclk ! href count: %u\n    ",
                 fstatOmni1.hrefCount, fstatOmni1.pclkNoHrefCount);
    for (uint16_t ii = 0; ii < fstatOmni1.hrefCount + 1; ii++) {
        debug.printf("%3u(%u) ", fstatOmni1.pclkCounts[ii],
                     (ii == 0) ? 0
                               : fstatOmni1.hrefStartTime[ii] -
                                     fstatOmni1.hrefStartTime[ii - 1]);
        if (!(ii & 0x0f))
            debug.print("\n    ");
    }
    debug.println();
}

/*****************************************************************/

typedef struct {
    const __FlashStringHelper *reg_name;
    uint8_t bank;
    uint16_t reg;
} OV2640_TO_NAME_t;

static const OV2640_TO_NAME_t OV2640_reg_name_table[] PROGMEM{
    {F(" QS"), 0, 0x44},
    {F(" HSIZE"), 0, 0x51},
    {F(" VSIZE"), 0, 0x52},
    {F(" XOFFL"), 0, 0x53},
    {F(" YOFFL"), 0, 0x54},
    {F(" VHYX"), 0, 0x55},
    {F(" DPRP"), 0, 0x56},
    {F(" TEST"), 0, 0x57},
    {F(" ZMOW"), 0, 0x5A},
    {F(" ZMOH"), 0, 0x5B},
    {F(" ZMHH"), 0, 0x5C},
    {F(" BPADDR"), 0, 0x7C},
    {F(" BPDATA"), 0, 0x7D},
    {F(" SIZEL"), 0, 0x8C},
    {F(" HSIZE8"), 0, 0xC0},
    {F(" VSIZE8"), 0, 0xC1},
    {F(" CTRL1"), 0, 0xC3},
    {F(" MS_SP"), 0, 0xF0},
    {F(" SS_ID"), 0, 0xF7},
    {F(" SS_CTRL"), 0, 0xF7},
    {F(" MC_AL"), 0, 0xFA},
    {F(" MC_AH"), 0, 0xFB},
    {F(" MC_D"), 0, 0xFC},
    {F(" P_CMD"), 0, 0xFD},
    {F(" P_STATUS"), 0, 0xFE},
    {F(" CTRLI"), 0, 0x50},
    {F(" CTRLI_LP_DP"), 0, 0x80},
    {F(" CTRLI_ROUND"), 0, 0x40},
    {F(" CTRL0"), 0, 0xC2},
    {F(" CTRL2"), 0, 0x86},
    {F(" CTRL3"), 0, 0x87},
    {F(" R_DVP_SP"), 0, 0xD3},
    {F(" R_DVP_SP_AUTO_MODE"), 0, 0x80},
    {F(" R_BYPASS"), 0, 0x05},
    {F(" R_BYPASS_DSP_EN"), 0, 0x00},
    {F(" R_BYPASS_DSP_BYPAS"), 0, 0x01},
    {F(" IMAGE_MODE"), 0, 0xDA},
    {F(" RESET"), 0, 0xE0},
    {F(" MC_BIST"), 0, 0xF9},
    {F(" BANK_SEL"), 0, 0xFF},
    {F(" BANK_SEL_DSP"), 0, 0x00},
    {F(" BANK_SEL_SENSOR"), 0, 0x01},
    {F(" GAIN"), 1, 0x00},
    {F(" COM1"), 1, 0x03},
    {F(" REG_PID"), 1, 0x0A},
    {F(" REG_VER"), 1, 0x0B},
    {F(" COM4"), 1, 0x0D},
    {F(" AEC"), 1, 0x10},
    {F(" CLKRC"), 1, 0x11},
    {F(" CLKRC_DOUBLE"), 1, 0x82},
    {F(" CLKRC_DIVIDER_MASK"), 1, 0x3F},
    {F(" COM10"), 1, 0x15},
    {F(" HREFST"), 1, 0x17},
    {F(" HREFEND"), 1, 0x18},
    {F(" VSTRT"), 1, 0x19},
    {F(" VEND"), 1, 0x1A},
    {F(" MIDH"), 1, 0x1C},
    {F(" MIDL"), 1, 0x1D},
    {F(" AEW"), 1, 0x24},
    {F(" AEB"), 1, 0x25},
    {F(" REG2A"), 1, 0x2A},
    {F(" FRARL"), 1, 0x2B},
    {F(" ADDVSL"), 1, 0x2D},
    {F(" ADDVSH"), 1, 0x2E},
    {F(" YAVG"), 1, 0x2F},
    {F(" HSDY"), 1, 0x30},
    {F(" HEDY"), 1, 0x31},
    {F(" REG32"), 1, 0x32},
    {F(" ARCOM2"), 1, 0x34},
    {F(" REG45"), 1, 0x45},
    {F(" FLL"), 1, 0x46},
    {F(" FLH"), 1, 0x47},
    {F(" COM19"), 1, 0x48},
    {F(" ZOOMS"), 1, 0x49},
    {F(" COM22"), 1, 0x4B},
    {F(" COM25"), 1, 0x4E},
    {F(" BD50"), 1, 0x4F},
    {F(" BD60"), 1, 0x50},
    {F(" REG5D"), 1, 0x5D},
    {F(" REG5E"), 1, 0x5E},
    {F(" REG5F"), 1, 0x5F},
    {F(" REG60"), 1, 0x60},
    {F(" HISTO_LOW"), 1, 0x61},
    {F(" HISTO_HIGH"), 1, 0x62},
    {F(" REG04"), 1, 0x04},
    {F(" REG08"), 1, 0x08},
    {F(" COM2"), 1, 0x09},
    {F(" COM2_STDBY"), 1, 0x10},
    {F(" COM3"), 1, 0x0C},
    {F(" COM7"), 1, 0x12},
    {F(" COM8"), 1, 0x13},
    {F(" COM8_DEFAULT"), 1, 0xC0},
    {F(" COM8_BNDF_EN"), 1, 0x20},
    {F(" COM8_AGC_EN"), 1, 0x04},
    {F(" COM8_AEC_EN"), 1, 0x01},
    {F(" COM9"), 1, 0x14},
    {F(" CTRL1_AWB"), 1, 0x08},
    {F(" VV"), 1, 0x26},
    {F(" REG32"), 1, 0x32},
};

#define CNT_REG_NAME_TABLE \
    (sizeof(OV2640_reg_name_table) / sizeof(OV2640_reg_name_table[0]))

#ifdef DEBUG_CAMERA_REG
void Debug_printCameraWriteRegister(uint8_t reg, uint8_t data) {
    static uint8_t showing_bank = 0;
    static uint16_t bank_1_starts_at = 0;
    uint16_t ii = CNT_REG_NAME_TABLE; // initialize to remove warning

    debug.printf("cameraWriteRegister(%x, %x)", reg, data);
    if (reg == 0xff)
        showing_bank = data; // remember which bank we are
    else {
        if (showing_bank == 0) {
            for (ii = 0; ii < CNT_REG_NAME_TABLE; ii++) {
                if (OV2640_reg_name_table[ii].bank != 0)
                    break;
                if (reg == OV2640_reg_name_table[ii].reg)
                    break;
            }
        } else {
            if (bank_1_starts_at == 0) {
                for (ii = bank_1_starts_at; ii < CNT_REG_NAME_TABLE; ii++) {
                    if (OV2640_reg_name_table[ii].bank != 0) {
                        bank_1_starts_at = ii;
                        break;
                    }
                }
            }
            for (ii = bank_1_starts_at; ii < CNT_REG_NAME_TABLE; ii++) {
                if (reg == OV2640_reg_name_table[ii].reg)
                    break;
            }
        }
    }
    if ((ii < CNT_REG_NAME_TABLE) && (reg == OV2640_reg_name_table[ii].reg))
        debug.printf(" - %s\n", OV2640_reg_name_table[ii].reg_name);
    else
        debug.println();
}

#endif

void OV2640::showRegisters(void) {
    debug.println("\n*** Camera Registers ***");
    cameraWriteRegister(0xFF, 0x00); // bank 0
    bool showing_bank_0 = true;
    for (uint16_t ii = 0; ii < (sizeof(OV2640_reg_name_table) /
                                sizeof(OV2640_reg_name_table[0]));
         ii++) {
        if ((OV2640_reg_name_table[ii].bank != 0) && showing_bank_0) {
            cameraWriteRegister(0xff, 0x01);
            showing_bank_0 = false;
        }
        uint8_t reg_value = cameraReadRegister(OV2640_reg_name_table[ii].reg);
        debug.printf("(%d) %s(%x): %u(%x)\n", OV2640_reg_name_table[ii].bank,
                     OV2640_reg_name_table[ii].reg_name,
                     OV2640_reg_name_table[ii].reg, reg_value, reg_value);
    }

    // Quick and dirty print out WIndows start and end:
    uint8_t reg32_reg = cameraReadRegister(REG32);
    uint8_t com1_reg = cameraReadRegister(COM1);

    uint16_t hstart = (cameraReadRegister(HREFST) << 3) | (reg32_reg & 0x7);
    uint16_t hstop =
        (cameraReadRegister(HREFEND) << 3) | ((reg32_reg >> 3) & 0x7);
    uint16_t vstart = (cameraReadRegister(VSTRT) << 2) | (com1_reg & 0x3);
    uint16_t vstop = (cameraReadRegister(VEND) << 2) | ((com1_reg >> 2) & 0x3);

    debug.printf(
        "Window Hor: (%u - %u) = %u * 2 = %u Vert: (%u - %u) = %u * 2 = %u\n",
        hstart, hstop, hstop - hstart, (hstop - hstart) * 2, vstart, vstop,
        vstop - vstart, (vstop - vstart) * 2);

    cameraWriteRegister(0xFF, 0x00); // bank 0

    uint8_t vhyx_reg = cameraReadRegister(VHYX);

    uint8_t hsize = cameraReadRegister(HSIZE) | ((vhyx_reg >> 3) & 1) << 8;
    uint8_t vsize = cameraReadRegister(VSIZE) | ((vhyx_reg >> 7) & 1) << 8;

    debug.printf("device\tSize H: %u * 4 = %u V: %u * 4 = %u\n", hsize,
                 hsize * 4, vsize, vsize * 4);

    uint16_t xoffl = cameraReadRegister(XOFFL) | ((vhyx_reg >> 0) & 0x7) << 8;
    uint16_t yoffl = cameraReadRegister(YOFFL) | ((vhyx_reg >> 4) & 0x7) << 8;
    debug.printf("\t offset H: %u V: %u \n", xoffl, yoffl);
}
