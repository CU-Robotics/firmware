#ifndef __CAMERA_H
#define __CAMERA_H

#include "common.h"
#include <Arduino.h>
#include <DMAChannel.h>
#include <FlexIO_t4.h>
#include <Wire.h>

#if __has_include("user_camera_pins.h")
#include "user_camera_pins.h"
#else
#include "default_camera_pins.h"
#endif

class ImageSensor : public FlexIOHandlerCallback {
  public:
    // ImageSensor();
    virtual ~ImageSensor() {}

    // must be called before Camera.begin()
    virtual void setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin,
                         uint8_t hsync_pin, uint8_t en_pin,
                         uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3,
                         uint8_t g4 = 0xff, uint8_t g5 = 0xff,
                         uint8_t g6 = 0xff, uint8_t g7 = 0xff,
                         uint8_t shutdn_pin = 0xff,
                         TwoWire &wire = Wire);
    virtual bool begin(framesize_t framesize = FRAMESIZE_QVGA,
                       int framerate = 30, bool use_gpio = false) = 0;
    virtual void end() = 0;
    virtual int reset() = 0;
    virtual void showRegisters(void) = 0;
    virtual void debug(bool debug_on) { _debug = debug_on; }
    virtual bool debug() { return _debug; }
    bool usingGPIO() { return _use_gpio; }
    camera_input_t cameraInput() { return _cameraInput; }

    // debug and experimenting support
    virtual uint8_t readRegister(uint8_t reg) { return (uint8_t)-1; }
    virtual bool writeRegister(uint8_t reg, uint8_t data) { return false; }
    virtual int setPixformat(pixformat_t pfmt) = 0;
    virtual pixformat_t getPixformat() { return (pixformat_t)_format; }
    virtual uint8_t setFramesize(framesize_t framesize) = 0;
    virtual uint8_t setFramesize(int w, int h) {
        return 0;
    } // some cameras don't support
    virtual bool setZoomWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
        return false;
    }

    virtual int setFramerate(int framerate) = 0;
    virtual int setBrightness(int level) = 0;
    virtual int setGainceiling(gainceiling_t gainceiling) = 0;
    virtual int setColorbar(int enable) = 0;
    virtual int setAutoGain(int enable, float gain_db,
                            float gain_db_ceiling) = 0;
    virtual int get_vt_pix_clk(uint32_t *vt_pix_clk) = 0;
    virtual int getGain_db(float *gain_db) = 0;
    virtual int getCameraClock(uint32_t *vt_pix_clk) = 0;
    virtual int setAutoExposure(int enable, int exposure_us) = 0;
    virtual int getExposure_us(int *exposure_us) = 0;
    virtual int setHmirror(int enable) = 0;
    virtual int setVflip(int enable) = 0;
    virtual uint8_t setMode(uint8_t Mode, uint8_t FrameCnt) = 0;
    virtual uint8_t cmdUpdate() = 0;
    virtual uint8_t loadSettings(camera_reg_settings_t settings) = 0;
    // HM01B0 only so if you use should return -1)
    virtual uint8_t getAE(ae_cfg_t *psAECfg) = 0;
    virtual uint8_t calAE(uint8_t CalFrames, uint8_t *Buffer,
                          uint32_t ui32BufferLen, ae_cfg_t *pAECfg) = 0;
    virtual uint16_t getModelid() = 0;
    virtual void captureFrameStatistics() = 0;

    virtual bool
    begin_omnivision(framesize_t resolution = FRAMESIZE_QVGA,
                     pixformat_t format = RGB565, int fps = 30,
                     int camera_name = OV7670,
                     bool use_gpio = false);        // Supported FPS: 1, 5, 10, 15, 30
    virtual void setSaturation(int saturation) = 0; // 0 - 255
    virtual void setHue(int hue) = 0;               // -180 - 180
    virtual void setContrast(int contrast) = 0;     // 0 - 127
    virtual void setGain(int gain) = 0;             // 0 - 255
    virtual void autoGain(int enable, float gain_db, float gain_db_ceiling) = 0;
    virtual void setExposure(int exposure) = 0; // 0 - 65535
    virtual void autoExposure(int enable) = 0;

    virtual int setAutoWhitebal(int enable, float r_gain_db, float g_gain_db,
                                float b_gain_db);

    // grab Frame functions
    //-------------------------------------------------------
    // Generic Read Frame base on _hw_config
    virtual size_t readFrame(void *buffer1, size_t cb1, void *buffer2 = nullptr,
                             size_t cb2 = 0); // give default one for now

    virtual void *readFrameReturnBuffer() { return _dma_last_completed_frame; }

    virtual size_t readImageSizeBytes() { return _dma_last_completed_image_size; }

    virtual void useDMA(bool f) { _fuse_dma = f; }
    virtual bool useDMA() { return _fuse_dma; }

    virtual void data4BitMode(bool f) { _fdata_4bit_mode = f; }
    virtual bool data4BitMode() { return _fdata_4bit_mode; }
    virtual bool dataPinsReversed() { return _fshifter_pins_reversed; }

    // normal Read mode
    // We will include default implementation, used by some/all of the current
    // ones.
    virtual size_t readFrameGPIO(void *buffer, size_t cb1 = (uint32_t)-1,
                                 void *buffer2 = nullptr, size_t cb2 = 0);
    virtual void readFrame4BitGPIO(void *buffer) = 0;

    // Have default implementations that simply call off to flexio or GPIO...
    virtual bool readContinuous(bool (*callback)(void *frame_buffer), void *fb1,
                                size_t cb1, void *fb2, size_t cb2);
    virtual void stopReadContinuous();

    // FlexIO is default mode for the camera
    virtual size_t readFrameFlexIO(void *buffer, size_t cb1 = (uint32_t)-1,
                                   void *buffer2 = nullptr, size_t cb2 = 0);

    virtual bool startReadFlexIO(bool (*callback)(void *frame_buffer),
                                 void *fb1, size_t cb1, void *fb2, size_t cb2);
    virtual bool stopReadFlexIO();

    // start off
    virtual size_t readFrameCSI(void *buffer, size_t cb1 = (uint32_t)-1,
                                void *buffer2 = nullptr, size_t cb2 = 0);

    virtual bool startReadCSI(bool (*callback)(void *frame_buffer), void *fb1,
                              size_t cb1, void *fb2, size_t cb2);

    virtual bool stopReadCSI();

    // helper function, that if you ask the CSI to not use DMA, may have
    // issues, so will convert over to use FlexIO3 to do this.
    // not sure if it needs to be virtual, but...
    virtual size_t readFrameCSI_use_FlexIO(void *buffer, size_t cb1 = (uint32_t)-1,
                                           void *buffer2 = nullptr, size_t cb2 = 0);

    // checks and if necessary changes the mode to either CSI or FlexIO
    bool changeCSIReadToFlexIOMode(bool flexio_mode);

    // Lets try a dma version.  Doing one DMA that is synchronous does not gain
    // anything So lets have a start, stop... Have it allocate 2 frame buffers
    // and it's own DMA buffers, with the option of setting your own buffers if
    // desired.
    virtual bool
    startReadFrameDMA(bool (*callback)(void *frame_buffer) = nullptr,
                      uint8_t *fb1 = nullptr, uint8_t *fb2 = nullptr) = 0;

    virtual void changeFrameBuffer(uint8_t *fbFrom, uint8_t *fbTo) = 0;

    virtual bool stopReadFrameDMA() = 0;

    virtual void setVSyncISRPriority(uint8_t priority);
    virtual void setDMACompleteISRPriority(uint8_t priority) = 0;

    virtual uint32_t frameCount() = 0; //{return _dma_frame_count;}

    // Set and retrieve read timeout
    uint32_t timeout() { return _timeout; }
    void timeout(uint32_t timeout_ms) { _timeout = timeout_ms; }

    // The width and height are the sizes of the data returned when you do a
    // frameRead. initialially it is the size of the resolution that was passed
    // into setFramesize however if you set a zoom window this is the size of
    // that window.
    virtual int16_t width(void) { return _width; }
    virtual int16_t height(void) { return _height; }
    // Save of the frame in width and height set by setFramesize
    virtual int16_t frameWidth(void) { return _frame_width; }
    virtual int16_t frameHeight(void) { return _frame_height; }
    virtual int16_t mode(void) = 0;

    // See which of these are used by which cameras.
    framesize_t framesize;
    camera_reg_settings_t settings;
    hw_config_t _hw_config;
    hw_carrier_t _hw_carrier;

    // FlexIO transfer variables and methods:
    static void dmaInterruptFlexIO();
    virtual void processDMAInterruptFlexIO();
    static void frameStartInterruptFlexIO();
    virtual void processFrameStartInterruptFlexIO();
    virtual void processDMAInterrupt() {}

    // Callback function for flexio
    bool call_back(FlexIOHandler *pflex);

    // default for Micromod
    // The hardware configure will replace both
    // the CSI and flexio configure.
    // it will check what the pins belong to...
    virtual bool hardware_configure();
    virtual bool flexio_configure();
    virtual bool csi_configure();
    bool checkForCSIPins();

    // move a few more methods here:
    virtual void beginXClk();
    virtual void endXClk();

    virtual void processCSIInterrupt();
    static void CSIInterrupt();

    // default for Teensy 4.1
    virtual bool csi_reset_dma(); // needed if something goes wrong.
    virtual void processFrameStartInterrupt() {};
    virtual bool supports4BitMode() { return false; }

    void dumpDMA_TCD(DMABaseClass *dmabc, const char *psz_title);

  protected:
    bool _debug = true;               // Should the camera code print out debug information?
    bool _fuse_dma = true;            // in some cameras should we use DMA or do the Io directly
    bool _fdata_4bit_mode = false;    // Some cameras only support 4 bit mode
    bool _use_gpio = false;           // set in the begin of some cameras
    bool _csi_in_flexio_mode = false; // should combine some of these.
    camera_input_t _cameraInput = CAMERA_INPUT_DEFAULT;
    uint32_t _timeout = 2000; // timeout in ms for a read

    // Note this all could be uint8_t..
    int _vsyncPin = CAMERAPIN_VSYNC;
    int _hrefPin = CAMERAPIN_HREF;
    int _pclkPin = CAMERAPIN_PLK;
    int _xclkPin = CAMERAPIN_XCLK;
    int _rst = CAMERAPIN_RST;
    int _rst_init = CAMERAPIN_RST_INIT;
    int _pwdn = CAMERAPIN_PWDN;
    int _pwdn_init = CAMERAPIN_PWDN_INIT;
    int _dPins[8] = {CAMERAPIN_D0, CAMERAPIN_D1, CAMERAPIN_D2, CAMERAPIN_D3,
                     CAMERAPIN_D4, CAMERAPIN_D5, CAMERAPIN_D6, CAMERAPIN_D7};

    int16_t _width;
    int16_t _height;
    int16_t _frame_width;
    int16_t _frame_height;
    int _bytesPerPixel;
    bool _grayscale = false;
    int _format = 0;

    TwoWire *_wire = &Wire;

    static DMAChannel _dmachannel;
    static DMASetting _dmasettings[10]; // For now lets have enough for two full
                                        // size buffers...
    volatile bool _dma_active = false;
    uint8_t *_frame_buffer_1 = nullptr;
    size_t _frame_buffer_1_size = 0;
    uint8_t *_frame_buffer_2 = nullptr;
    size_t _frame_buffer_2_size = 0;
    // void* _last_frame_buffer_returned = nullptr;
    FlexIOHandler *_pflex;
    IMXRT_FLEXIO_t *_pflexio;
    uint8_t _fshifter = 0xff;
    uint8_t _fshifter_mask;
    uint8_t _ftimer;
    uint8_t _dma_source;
    uint8_t _fshifter_jpeg = 0xff; // if jpeg have we claimed shifter?
    uint8_t _fshifter_jpeg_mask = 0;
    bool _fshifter_pins_reversed = false;
    int _xclk_freq = 12;

    volatile uint32_t *_vsyncPort;
    uint32_t _vsyncMask;
    volatile uint32_t *_hrefPort;
    uint32_t _hrefMask;
    volatile uint32_t *_pclkPort;
    uint32_t _pclkMask;

    bool (*_callback)(void *frame_buffer) = nullptr;
    uint32_t _dma_frame_count;
    uint8_t *_dma_last_completed_frame;
    size_t _dma_last_completed_image_size;

    enum {
        DMASTATE_INITIAL = 0,
        DMASTATE_RUNNING,
        DMASTATE_STOP_REQUESTED,
        DMA_STATE_STOPPED,
        DMA_STATE_FRAME_ERROR,
        DMA_STATE_ONE_FRAME
    };
    volatile uint8_t _dma_state = DMASTATE_INITIAL;
    static ImageSensor *active_dma_camera;

  private:
};

class Camera {
  public:
    Camera(ImageSensor &sensor);

    /**
     * Configures Teensy pins to use for device and must be called
     * camera.begin(...).
     *
     * Note: Pin number: 0xff means not used, 0xfe says don't change
     * Pin order: XCLK (MCLK), PCLK, VSYNCH, HSYNC,  ENABLE,
     *            Data Pins(D0 - D8)
     *           SHUTDN,
     *            I2C bus (Wire used by Default)
     * 1. FLEXIO pins are used for data pins (consectutive), PCLK,
     *    and HSYNCH.
     * 2. PWM pin should be used for XCLK
     *
     */
    void setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin,
                 uint8_t hsync_pin, uint8_t en_pin,
                 uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3,
                 uint8_t g4 = 0xff, uint8_t g5 = 0xff,
                 uint8_t g6 = 0xff, uint8_t g7 = 0xff,
                 uint8_t shutdn_pin = 0xff,
                 TwoWire &wire = Wire);

    /**
     * For HIMAX camers, begin(...).
     *     1. framesize:    allowable framesize enumerator.
     *     2. framerate:    supported framerate, or any framerate if not
     *        supported.
     *     3. GPIO boolean: identify whether lib should use GPIO or FLEXIO.
     *
     * Returns 0 if successful, nonzero if fails.
     */
    bool begin(framesize_t framesize = FRAMESIZE_QVGA, int framerate = 30,
               bool use_gpio = false);

    /**
     * If supported this method with end external clock and disables I2C.
     */
    void end();

    /**
     * This method will reset the camera to default register settings.
     */
    int reset();
    /**
     * Shows current registers and values. Can be called anytime.
     */
    void showRegisters(void);

    /**
     * Enables or disables debug messages in the library.
     */
    void debug(bool debug_on);

    /**
     * Returns current state of debug messages as a boolean, true or false
     */
    bool debug();

    /**
     * Returns as a boolean whether the library is configured to use
     * GPIO or FLEXIO
     */
    bool usingGPIO();

    /**
     * Returns if GPIO or FlexIO or CSI is being used by the camera
     * Note: we should probably combine the usage of using GPIO...
     * inputs: <none>
     * returns:
     *   CAMERA_INPUT_DEFAULT = 0,
     *   CAMERA_INPUT_FLEXIO,
     *   CAMERA_INPUT_CSI,
     *   CAMERA_INPUT_GPIO,
     *   CAMERA_INPUT_GPIO4
     */
    camera_input_t cameraInput();

    /**
     * Sets pixel format for the camera using the camera format enumerator.
     * HIMAX Cameras: Only GRAYSCALE IS SUPPORTED.
     * For other cameras RGB565 is typically used for displaying images.
     * The OV2640/5640 are the only cameras that support JPEG fromat.
     */
    int setPixformat(pixformat_t pfmt);
    pixformat_t getPixformat();

    /**
     * Set the resolution of the image sensor.
     *
     * This has no effect on cameras that do not support variable
     * resolutions.
     * Param: resolution, The desired resolution, as defined in the resolution
     * enumerator.
     * Returns: 0 on success, non-zero on failure.
     */
    uint8_t setFramesize(framesize_t framesize);

    /**
     * Sets the framesize to specified width and height in number of pixels
     *
     * This has no effect on cameras that do not support variable
     * resolutions.
     * Returns: 0 on success, non-zero on failure.
     */
    uint8_t setFramesize(int w, int h);

    /**
     * Sets window zoom for those cameras that support camera zoom function
     *
     * This has no effect on cameras that do now support camera zoom. Currently
     * supported by the GC2145 and OV2640.
     * INPUT:
     * 1. x, y - origin starting point for zooming.
     *           if these are -1 will center the frame with last w, h
     * 2. w, h - image width and height to display.
     *           if these values are -1 use previous width and height
     * all units are in pixels. The defined rectangle must be fully contained
     *           within the bounds of the current frame size.
     * Returns: true if success or false if not.
     */
    bool setZoomWindow(uint16_t x = -1, uint16_t y = -1, uint16_t w = -1,
                       uint16_t h = -1);

    /**
     * Sets the camera framerate for those cameras that support framerate.
     *
     * This has no effect on cameras that do now support framerate.
     * INPUT: Framerate in frames per second (fps).
     * Typically 15 or 30 fps.
     * Returns: non-zero if fails.
     * NOTE:  Only HIMAX cameras support framerate.
     */
    int setFramerate(int framerate);

    /**
     * Sets the image brightness level
     *
     * This function is not supported by the GC2145.
     * INPUT for OV2640: -2 to +2 in steps of 1.
     * INPUT for OV7670/OV7675: 0 to 255
     * INPUT for HIMAX: 0 to 3
     *
     * Returns:
     */
    int setBrightness(int level);

    /**
     * Sets Gain Ceiling based on gain enumerator.
     *
     * This has no effect on cameras that do not support GainCeiling.
     *
     * INPUT: Based on gain ceiling enumerator.
     * Returns: non-zero if fails.
     */
    int setGainceiling(gainceiling_t gainceiling);

    /**
     * Enables color bar to be displayed.
     *
     * For OV2640 and HIMAX:
     * INPUT: 0 - off, 1 - on.
     * For GC2145 different testpatterns are available.
     *      GC2145_DISABLED = 0,.
     *       GC2145_COLOR_BARS,.
     *       GC2145_UXGA_COLOR_BARS,.
     *       GC2145_SKIN_MAP,.
     *       GC2145_SOLID_BLACK,.
     *       GC2145_SOLID_LIGHT_GRAY,.
     *       GC2145_SOLID_GRAY,.
     *       GC2145_SOLID_DARK_GRAY,.
     *       GC2145_SOLID_WHITE,.
     *       GC2145_SOLID_RED,.
     *       GC2145_SOLID_GREEN,.
     *       GC2145_SOLID_BLUE,.
     *       GC2145_SOLID_YELLOW,.
     *       GC2145_SOLID_CYAN,.
     *       GC2145_SOLID_MAGENTA.
     * Not available for OV767X cameras
     * RETURNS:
     */
    int setColorbar(int enable);

    /**
     * Sets Automatic Gain based on db's.
     *
     * Supported by all cameras except GC2145.
     * Input:
     *     1. int enable - enables auto gain(1 enable, 0 disable).
     *     2. float gain_db = gain in db's.
     *     3. float gain_ceiling in dbs.
     * RESULT: Non-zero if it fails to set selected value.
     */
    int setAutoGain(int enable, float gain_db, float gain_db_ceiling);

    /**
     * Gets clock settings
     *
     * This has no effect on cameras that do not support this.
     */
    int get_vt_pix_clk(uint32_t *vt_pix_clk);
    int getCameraClock(uint32_t *vt_pix_clk);

    /**
     * Gets current gain in dbs.
     *
     * Not supported on OV767X and GC2145 cameras.
     */
    int getGain_db(float *gain_db);

    /**
     * Sets Automatic Exposure in microseconds.
     *
     * Not supported on the GC2145.
     * RESULT: Non-zero if it fails to set selected value.
     */
    int setAutoExposure(int enable, int exposure_us);

    /**
     * Returns current Exposure in microseconds.
     *
     * Not supported on the GC2145.
     * RESULT: Current exposure in microseconds.
     */
    int getExposure_us(int *exposure_us);

    /**
     * Enable or disable Horizontal Mirror of the camera.
     *
     * When enabled, pixels go from right to left instead
     * of left to right.  When used with setVflip, it makes
     * as if the camera is rotated 180 degrees
     *
     * Input: true or false.
     * RESULT: Non-zero if it fails to set selected value.
     */
    int setHmirror(int enable);

    /**
     * Enable or disable camera Vertical Flip.
     *
     * When enabled, pixels go from bottom to top instead
     * of top to bottom.  When used with setHmirror, it makes
     * as if the camera is rotated 180 degrees
     *
     * Input: true or false.
     * RESULT: Non-zero if it fails to set selected value.
     */
    int setVflip(int enable);

    /**
     * Sets Framemode for HIMAX Cameras only.
     *
     * This has no effect on cameras other than HIMAX.
     * RESULT: Non-zero if it fails to set selected value.
     */
    uint8_t setMode(uint8_t Mode, uint8_t FrameCnt);

    /**
     * Excutes a register update for HIMAX cameras only.
     *
     * RESULT: Non-zero if it fails to set selected value.
     */
    uint8_t cmdUpdate();

    /**
     * Loads HIMAX Camera configuration settings.
     *
     * RESULT: Non-zero if it fails to set selected value.
     */
    uint8_t loadSettings(camera_reg_settings_t settings);

    // HM01B0 only so if you use should return -1)
    uint8_t getAE(ae_cfg_t *psAECfg);
    uint8_t calAE(uint8_t CalFrames, uint8_t *Buffer, uint32_t ui32BufferLen,
                  ae_cfg_t *pAECfg);
    uint16_t getModelid();

    // debug and experimenting support
    uint8_t readRegister(uint8_t reg);
    bool writeRegister(uint8_t reg, uint8_t data);

    /**
     * For Omnivision and GalaxyCore cameras.
     *     1. framesize:    allowable framesize enumerator.
     *     2. format:       format enumerator such as RGB565 for the camera.
     *     3. framerate:    supported framerate, or any framerate if not
     * supported
     *     4. Camera Name:  OV2640, OV7670, OV7675 or GC2145.
     *     5. GPIO boolean: identify whether lib should use GPIO or FLEXIO.
     *
     * Returns 0 if successful, nonzero if fails.
     */
    bool begin(framesize_t resolution = FRAMESIZE_QVGA,
               pixformat_t format = RGB565, int fps = 30,
               int camera_name = OV7670,
               bool use_gpio = false); // Supported FPS: 1, 5, 10, 15, 30

    /**
     * Sets Saturation levels for Omnivision Cameras Only.
     *
     * OV2640 Range: -2 to +2.
     * OV767X Range: 0 - 255.
     * No return value.
     */
    void setSaturation(int saturation); // 0 - 255

    /**
     * Sets the Hue for the OV767X Cameras ONLY.
     * Range: -180 to +180.
     * No return value.
     */
    void setHue(int hue); // -180 - 180

    /**
     * Sets Image Contrast.
     *
     * OV2640 Range: -2 to +2.
     * OV767X Range: 0 - 127.
     * No return value.
     */
    void setContrast(int contrast); // 0 - 127

    /**
     * Sets camera gain for Omnivision cameras Only.
     *
     * RANGE: 0 - 255 for OV767X.
     * RANGE: 0 -  30 for OV2640.
     * No return value.
     */
    void setGain(int gain); // 0 - 255
    void autoGain(int enable, float gain_db, float gain_db_ceiling);

    /**
     * Sets exposure level for OV767x cameras ONLY.
     *
     * RANGE: 0 - 65535.
     */
    void setExposure(int exposure); // 0 - 65535

    /**
     * Enables Auto Exposure for Omnivision cameras Only.
     *
     * Input: 1 to enable, 0 to disable for OV767x camera
     * Input for OV2640: 0 - 4 AE levels.
     * RESULT: no return value
     */
    void autoExposure(int enable);

    /**
     * Sets Auto Whitebalance based on RGB.
     *
     * Only supported by the OV2640 camera.
     * INPUTS:.
     *   1. enable, 1 to enable, 0 to disable.
     *   2. r_gain_db - red gain in dbs.
     *   3. g_gain_db - red gain in dbs.
     *   4. b_gain_db - red gain in dbs.
     *
     * Returns 0 if successful, nonzero if fails.
     */
    int setAutoWhitebal(int enable, float r_gain_db, float g_gain_db,
                        float b_gain_db);

    // grab Frame functions

    /**
     * Read one Frame from the camera using the current settings
     * This method allows you to pass in two buffers and size of
     * buffers. This can be important when it is very possible that
     * you do not have enough room in either DTCM or DMAMEM to hold
     * one whole frame.  For example: OV2640, OV7670, OV7675 or GC2145
     * cameras allow you to read in a VGA size (640 by 480) with 2 bytes
     * per pixel this requires a buffer size of at least: 614400 bytes
     * which is larger than either memory region.
     *
     * Inputs:
     *     buffer1 - pointer to first buffer
     *     cb1 - size of buffer 1 in bytes
     *     buffer2 - pointer to optional second buffer
     *     cb2 - size of second optional buffer
     *
     * Returns: count of bytes returned from the camera, 0 if error
     */
    size_t readFrame(void *buffer1, size_t cb1, void *buffer2 = nullptr,
                     size_t cb2 = 0);

    /**
     * Return which frame buffer was used in the last read.  This is
     * mostly important with the CSI setup as CSI wants to alternate
     * from buffer 1 to buffer 2
     * Inpute: <none>
     * return: pointer to last camera buffer that was completed
     */
    void *readFrameReturnBuffer();

    /**
     * Return how many bytes were used in that last frame buffer.
     * Note: in most cases this will be width*hight*bytes_per_pixel.
     * But: JPEG images are different.
     * Inpute: <none>
     * return: count of bytes to last image in the camera buffer
     */
    size_t readImageSizeBytes();

    /**
     * Tells some readFrameFlexIO implementions if they should use DMA
     * or not.
     * Input: bool - yes or no
     * Returns: none
     */
    void useDMA(bool f);

    /**
     * Returns the current state of if FlexIO implementations should
     * use DMA or not.
     * Returns: true (default) if they are configured to use DMA
     */
    bool useDMA();

    /**
     * Some cameras support 4 data bit mode.  Some like
     * the Arduino HM01b0 only supports 4 bit mode.
     * or not.
     * Input: bool - yes or no
     * Returns: none
     */
    void data4BitMode(bool f);

    /**
     * Returns True if we are in 4 bit mode.
     * use DMA or not.
     * Returns: true (default) if they are configured to use DMA
     */
    bool data4BitMode();

    /**
     * Sometimes we run into FlexIO pins that are reversed
     * That is instead of D1 == D0 + 1 and D2 == D0 + 2...
     * We have D1 == D0 - 1 and D23 == D0 - 2 ...
     * Most likely case: use FlexIO on CSI pins.
     * Sometimes the sketch needs to know this es for example
     * 4 bit mode, the sketch may need to do something special.
     * Inputs: None
     * Returns: true if the flexio data pins are reversed.
     */
    bool dataPinsReversed();

    // normal Read mode

    /**
     * Read one Frame using GPIO from the camera using the current settings.
     * This method allows you to pass in two buffers and size of
     * buffers. This can be important when it is very possible that
     * you do not have enough room in either DTCM or DMAMEM to hold
     * one whole frame.  For example: OV2640, OV7670, OV7675 or GC2145
     * cameras allow you to read in a VGA size (640 by 480) with 2 bytes
     * per pixel this requires a buffer size of at least: 614400 bytes
     * which is larger than either memory region.
     *
     * Inputs:
     *     buffer1 - pointer to first buffer
     *     cb1 - size of buffer 1 in bytes
     *     buffer2 - pointer to optional second buffer
     *     cb2 - size of second optional buffer
     *
     * Returns: count of bytes returned from the camera, 0 if error
     */
    size_t readFrameGPIO(void *buffer, size_t cb1 = (uint32_t)-1,
                         void *buffer2 = nullptr, size_t cb2 = 0);

    /**
     * Read one Frame using GPIO from the camera using the current settings
     * in 4bit mode.  ONLY APPLIES TO HIMAX CAMERAS.
     *
     * Inputs:
     *     buffer1 - pointer to first buffer
     *
     * Returns: count of bytes returned from the camera, 0 if error
     */
    void readFrame4BitGPIO(void *buffer);

    /**
     * Start a continuous read operation on the camera.
     * Currently this is only supported with the FLEXIO DMA implementation
     * Both fb1 and fb2 need to large enough to hold a complete frame.
     * Your callback function will be called as each frame is completed, with
     * a pointer to the buffer that was just filled.
     * Inputs:
     *    callback - pointer to callback function that is called from DMA
     *               completion interrupt.
     *    fb1 - pointer to first frame buffer
     *    cb1 - size of that buffer
     *    fb2 - pointer to second buffer
     *    cb2 - size of the second buffer
     *  Returns: true if the continuous read was started.
     */
    bool readContinuous(bool (*callback)(void *frame_buffer), void *fb1,
                        size_t cb1, void *fb2, size_t cb2);

    /**
     * Stops a continuous read operation started by the readContinuous
     * call.  There are no inputs or return values.
     */
    void stopReadContinuous();

    // FlexIO is default mode for the camera
    /**
     * Read one Frame using FLEXIO from the camera using the current settings
     * This method allows you to pass in two buffers and size of
     * buffers. This can be important when it is very possible that
     * you do not have enough room in either DTCM or DMAMEM to hold
     * one whole frame.  For example: OV2640, OV7670, OV7675 or GC2145
     * cameras allow you to read in a VGA size (640 by 480) with 2 bytes
     * per pixel this requires a buffer size of at least: 614400 bytes
     * which is larger than either memory region.
     *
     * Inputs:
     *     buffer1 - pointer to first buffer
     *     cb1 - size of buffer 1 in bytes
     *     buffer2 - pointer to optional second buffer
     *     cb2 - size of second optional buffer
     *
     * Returns: count of bytes returned from the camera, 0 if error    size_t
     * readFrameGPIO(void *buffer, size_t cb1 = (uint32_t)-1, void *buffer2 =
     * nullptr, size_t cb2 = 0);
     */
    size_t readFrameFlexIO(void *buffer, size_t cb1 = (uint32_t)-1,
                           void *buffer2 = nullptr, size_t cb2 = 0);

    bool startReadFlexIO(bool (*callback)(void *frame_buffer), void *fb1,
                         size_t cb1, void *fb2, size_t cb2);
    bool stopReadFlexIO();

    // CSI may be default on some cameras
    /**
     * Read one Frame using CSI from the camera using the current settings
     * This method allows you to pass in two buffers and size of
     * buffers. This can be important when it is very possible that
     * you do not have enough room in either DTCM or DMAMEM to hold
     * one whole frame.  For example: OV2640, OV7670, OV7675 or GC2145
     * cameras allow you to read in a VGA size (640 by 480) with 2 bytes
     * per pixel this requires a buffer size of at least: 614400 bytes
     * which is larger than either memory region.
     *
     * Inputs:
     *     buffer1 - pointer to first buffer
     *     cb1 - size of buffer 1 in bytes
     *     buffer2 - pointer to optional second buffer
     *     cb2 - size of second optional buffer
     *
     * Returns: count of bytes returned from the camera, 0 if error    size_t
     * readFrameGPIO(void *buffer, size_t cb1 = (uint32_t)-1, void *buffer2 =
     * nullptr, size_t cb2 = 0);
     */
    size_t readFrameCSI(void *buffer, size_t cb1 = (uint32_t)-1,
                        void *buffer2 = nullptr, size_t cb2 = 0);

    bool startReadCSI(bool (*callback)(void *frame_buffer), void *fb1,
                      size_t cb1, void *fb2, size_t cb2);
    bool stopReadCSI();

    // Lets try a dma version.  Doing one DMA that is synchronous does not gain
    // anything So lets have a start, stop... Have it allocate 2 frame buffers
    // and it's own DMA buffers, with the option of setting your own buffers if
    // desired.
    bool startReadFrameDMA(bool (*callback)(void *frame_buffer) = nullptr,
                           uint8_t *fb1 = nullptr, uint8_t *fb2 = nullptr);

    void changeFrameBuffer(uint8_t *fbFrom, uint8_t *fbTo);

    bool stopReadFrameDMA();

    void captureFrameStatistics();

    /**
     * Sets the VSYNCH priority for HIMAX Cameras ONLY.
     *
     * RANGE: 0 - 255, lower is higher priority for interrupt.
     * RETURN: no return
     */
    void setVSyncISRPriority(uint8_t priority);

    /**
     * Sets the DMA Complete Priority for HIMAX Cameras Only.
     *
     * RANGE: 0 - 255, lower is higher priority for interrupt.
     * RETURN: no return
     */
    void setDMACompleteISRPriority(uint8_t priority);

    uint32_t frameCount(); //{return _dma_frame_count;}

    /**
     * Return the width of the frame date that is returned by calls like
     * readFrame.  In most cases this is the width of the resolution that
     * passed into the begin methods or to setFrameSize.  However some of
     * the cameras cush as the OV2640 allow you to set a zoom window into
     * the resolution, and the width will return the width of the actual
     * data that is to be returned.
     *
     * Returns: width in pixels
     */
    int16_t width(void);

    /**
     * Return the height of the frame date that is returned by calls like
     * readFrame.  In most cases this is the height of the resolution that
     * passed into the begin methods or to setFrameSize.  However some of
     * the cameras cush as the OV2640 allow you to set a zoom window into
     * the resolution, and the width will return the height of the actual
     * data that is to be returned.
     *
     * Returns: height in pixels
     */
    int16_t height(void);

    /**
     * Return the width of the current frame resolution
     * In most cases this is the same value as the width() method.
     * However if if a zoom Window is active.  This call will continue
     * to return the width of the frame size, from the last call to
     * setFramesize.
     *
     * Returns: frame width in pixels
     */
    int16_t frameWidth(void);

    /**
     * Return the height of the current frame resolution
     * In most cases this is the same value as the height() method.
     * However if if a zoom Window is active.  This call will continue
     * to return the height of the frame size, from the last call to
     * setFramesize.
     *
     * Returns: frame height in pixels
     */
    int16_t frameHeight(void);

    int16_t mode(void);

    // set and retrieve read timeout
    uint32_t timeout();
    void timeout(uint32_t timeout_ms);

  private:
    ImageSensor *sensor; /// Pointer to the camera sensor
};
#endif
