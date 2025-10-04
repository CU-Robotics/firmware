#include <stdint.h>
typedef enum {
    PIXFORMAT_INVALID = 0,
    PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    PIXFORMAT_BAYER,     // 1BPP/RAW
    YUV422,
    RGB444,
    RGB565,
    BAYER,
    GRAYSCALE,
    JPEG,
} pixformat_t;

typedef enum {
    // VGA Resolutions
    FRAMESIZE_VGA = 0,
    FRAMESIZE_QQVGA, // 160x120
    FRAMESIZE_QVGA,  // 320x240
    FRAMESIZE_480X320,
    FRAMESIZE_320X320, // 320x320
    FRAMESIZE_QVGA4BIT,
    FRAMESIZE_QCIF,
    FRAMESIZE_CIF,
    FRAMESIZE_SVGA, // 800, 600
    FRAMESIZE_UXGA, // 1500, 1200

    FRAMESIZE_96X96,   // 96x96
    FRAMESIZE_HQVGA,   // 240x176
    FRAMESIZE_240X240, // 240x240
    FRAMESIZE_HVGA,    // 480x320
    FRAMESIZE_XGA,     // 1024x768
    FRAMESIZE_HD,      // 1280x720
    FRAMESIZE_SXGA,    // 1280x1024
    // 3MP Sensors
    FRAMESIZE_FHD,   // 1920x1080
    FRAMESIZE_P_HD,  //  720x1280
    FRAMESIZE_P_3MP, //  864x1536
    FRAMESIZE_QXGA,  // 2048x1536
    // 5MP Sensors
    FRAMESIZE_QHD,   // 2560x1440
    FRAMESIZE_WQXGA, // 2560x1600
    FRAMESIZE_P_FHD, // 1080x1920
    FRAMESIZE_QSXGA, // 2560x1920
    
    FRAMESIZE_1024X600, //1024x600

    FRAMESIZE_INVALID,
} framesize_t;

typedef enum {
    CAMERA_INPUT_DEFAULT = 0,
    CAMERA_INPUT_FLEXIO,
    CAMERA_INPUT_CSI,
    CAMERA_INPUT_GPIO,
    CAMERA_INPUT_GPIO4
} camera_input_t;

typedef enum {
    GAINCEILING_2X,
    GAINCEILING_4X,
    GAINCEILING_8X,
    GAINCEILING_16X,
    GAINCEILING_32X, // greater that 32 for OV2640
    GAINCEILING_64X,
    GAINCEILING_128X,
} gainceiling_t;

typedef enum {
    LOAD_DEFAULT_REGS,
    LOAD_WALKING1S_REG,
    LOAD_SHM01B0INIT_REGS,
    LOAD_320x240,
} camera_reg_settings_t;

typedef enum {
    TEENSY_MICROMOD_FLEXIO_8BIT = 0,
    TEENSY_MICROMOD_FLEXIO_4BIT,
} hw_config_t;

typedef enum {
    SPARKFUN_ML_CARRIER = 0,
    PJRC_CARRIER,
} hw_carrier_t;

typedef struct {
    uint8_t ui8AETargetMean;
    uint8_t ui8AEMinMean;
    uint8_t ui8ConvergeInTh;
    uint8_t ui8ConvergeOutTh;
    uint8_t ui8AEMean;
} ae_cfg_t;

typedef struct {
    uint8_t ui8IntegrationH;
    uint8_t ui8IntegrationL;
    uint8_t ui8AGain;
    uint8_t ui8DGain_H;
    uint8_t ui8DGain_L;
} hm01b0_snr_expo_gain_ctrl_t;

typedef enum {
    HIMAX_ERR_OK = 0x00,
    HIMAX_ERR_AE_NOT_CONVERGED,
    HIMAX_ERR_PARAMS,
    HIMAX_NUM_ERR
} status_e;

enum { OV7670 = 0,
       OV7675 = 1,
       GC2145a = 2,
       OV2640a = 3,
       OV5640a = 4 };

// for 0V2640 only
typedef enum {
    NOEFFECT,
    NEGATIVE,
    BW,
    REDDISH,
    GREENISH,
    BLUEISH,
    RETRO,
    OVEREXPOSURE,
    SOLARIZE
} sde_t;