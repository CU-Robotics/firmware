//=============================================================================
// Default implementations for some of the ImageSensor class
//=============================================================================

#include "Teensy_Camera.h"
#include "teensy_csi_support.h"

#define debug Serial

//#define DEBUG_CAMERA
//  #define DEBUG_CAMERA_VERBOSE
//#define DEBUG_FLEXIO
// #define USE_DEBUG_PINS

ImageSensor *ImageSensor::active_dma_camera = nullptr;
DMAChannel ImageSensor::_dmachannel;
DMASetting ImageSensor::_dmasettings[10];

//#define USE_DEBUG_PINS_TIMING

#ifdef ARDUINO_TEENSY41
#define DBG_TIMING_PIN 2
#else
#define DBG_TIMING_PIN 0
#endif

#ifdef USE_DEBUG_PINS_TIMING
#define DBGdigitalWriteFast digitalWriteFast
#define DBGdigitalToggleFast digitalToggleFast
#else
static inline void DBGdigitalWriteFast(uint8_t pin, uint8_t val)
    __attribute__((always_inline, unused));
static inline void DBGdigitalWriteFast(uint8_t pin, uint8_t val) {}
static inline void DBGdigitalToggleFast(uint8_t pin)
    __attribute__((always_inline, unused));
static inline void DBGdigitalToggleFast(uint8_t pin) {}
#endif

void ImageSensor::setPins(uint8_t mclk_pin, uint8_t pclk_pin, uint8_t vsync_pin, uint8_t hsync_pin,
                          uint8_t en_pin,
                          uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3, uint8_t g4,
                          uint8_t g5, uint8_t g6, uint8_t g7,
                          uint8_t shutdn_pin, TwoWire &wire) {
#ifdef DEBUG_CAMERA
    debug.printf("void ImageSensor::setPins(MC:%u PC:%u vs:%u hr:%u en:%u, DP: "
                 "%u %u %u %u %u %u %u %u sd:%u W:%p)\n",
                 mclk_pin, pclk_pin, vsync_pin, hsync_pin, en_pin,
                 g0, g1, g2, g3, g4, g5, g6, g7, shutdn_pin, &wire);
#endif
#define NOCHANGE_PIN 0xfe

    if (mclk_pin != NOCHANGE_PIN)
        _xclkPin = mclk_pin;
    if (pclk_pin != NOCHANGE_PIN)
        _pclkPin = pclk_pin;
    if (vsync_pin != NOCHANGE_PIN)
        _vsyncPin = vsync_pin;
    if (hsync_pin != NOCHANGE_PIN)
        _hrefPin = hsync_pin;
    if (en_pin != NOCHANGE_PIN)
        _rst = en_pin;
    if (g0 != NOCHANGE_PIN)
        _dPins[0] = g0;
    if (g1 != NOCHANGE_PIN)
        _dPins[1] = g1;
    if (g2 != NOCHANGE_PIN)
        _dPins[2] = g2;
    if (g3 != NOCHANGE_PIN)
        _dPins[3] = g3;
    if (g4 != NOCHANGE_PIN)
        _dPins[4] = g4;
    if (g5 != NOCHANGE_PIN)
        _dPins[5] = g5;
    if (g6 != NOCHANGE_PIN)
        _dPins[6] = g6;
    if (g7 != NOCHANGE_PIN)
        _dPins[7] = g7;
    if (shutdn_pin != NOCHANGE_PIN)
        _pwdn = shutdn_pin;
#ifdef DEBUG_CAMERA
    debug.printf("updated pins (MC:%u PC:%u vs:%u hr:%u en:%u DP: "
                 "%u %u %u %u %u %u %u %u sd:%u W:%p)\n",
                 _xclkPin, _pclkPin, _vsyncPin, _hrefPin, _rst, _dPins[0], _dPins[1], _dPins[2],
                 _dPins[3], _dPins[4], _dPins[5], _dPins[6], _dPins[7],
                 _pwdn, &wire);
#endif

    _wire = &wire;

    if (g4 == 0xff) {
        _hw_config = TEENSY_MICROMOD_FLEXIO_4BIT;
    } else {
        _hw_config = TEENSY_MICROMOD_FLEXIO_8BIT;
    }
}

size_t ImageSensor::readFrame(void *buffer1, size_t cb1, void *buffer2,
                              size_t cb2) {
    // if (!_use_gpio) { 
    //     if (_cameraInput == CAMERA_INPUT_CSI)
    //         return readFrameCSI(buffer1, cb1, buffer2, cb2);
    //     else
    //         return readFrameFlexIO(buffer1, cb1, buffer2, cb2);
    // } else {
    //     if (_hw_config == TEENSY_MICROMOD_FLEXIO_4BIT) {
    //         readFrame4BitGPIO(buffer1);
    //         return cb1; // not sure what to return here yet...
    //     }
    //     return readFrameGPIO(buffer1, cb1, buffer2, cb2);
    // }

    // optimization, since we will always use CSI
    return readFrameCSI(buffer1, cb1, buffer2, cb2);
}

// The read and stop continuous typically just call off to the FlexIO
bool ImageSensor::readContinuous(bool (*callback)(void *frame_buffer),
                                 void *fb1, size_t cb1, void *fb2, size_t cb2) {
    if (_debug)
        debug.printf("readContinuous called(%p, %p, %u, %p, %u): %u\n", callback, fb1, cb1, fb2, cb2, _cameraInput);
    if (_cameraInput == CAMERA_INPUT_CSI)
        return startReadCSI(callback, fb1, cb1, fb2, cb2);
    else
        return startReadFlexIO(callback, fb1, cb1, fb2, cb2);
}

void ImageSensor::stopReadContinuous() {
    if (_cameraInput == CAMERA_INPUT_CSI)
        stopReadCSI();
    else
        stopReadFlexIO();
}

//=============================================================================
// FlexIO support
//=============================================================================

size_t ImageSensor::readFrameFlexIO(void *buffer, size_t cb1, void *buffer2,
                                    size_t cb2) {
    uint32_t frame_size_bytes = _width * _height * _bytesPerPixel;
    if (_debug)
        debug.printf("$$ImageSensor::readFrameFlexIO(%p, %u, %p, %u, %u, %u) %u\n",
                     buffer, cb1, buffer2, cb2, _fuse_dma, _hw_config, frame_size_bytes);
    DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);

    if (_format == pixformat_t::JPEG) {
        frame_size_bytes = frame_size_bytes / 5;
    } else if ((cb1 + cb2) < frame_size_bytes) {
        if (_debug)
            debug.println("Error: buffers are too small");
        DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);
        return 0; // not enough to hold normal images...
    }

    // flexio_configure(); // one-time hardware setup
    // wait for VSYNC to go high and then low with a sort of glitch filter
    elapsedMillis emWaitSOF;
    elapsedMicros emGlitch;
    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);

    for (;;) {
        if (emWaitSOF > _timeout) {
            if (_debug)
                debug.println("Timeout waiting for Start of Frame");
            return 0;
        }
        while ((*_vsyncPort & _vsyncMask) == 0) {
            if (emWaitSOF > _timeout) {
                if (_debug)
                    debug.println(
                        "Timeout waiting for rising edge Start of Frame");
                return 0;
            }
        }
        emGlitch = 0;
        while ((*_vsyncPort & _vsyncMask) != 0) {
            if (emWaitSOF > _timeout) {
                if (_debug)
                    debug.println(
                        "Timeout waiting for falling edge Start of Frame");
                return 0;
            }
        }
        if (emGlitch > 5)
            break;
    }
    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;
    uint32_t *p = (uint32_t *)buffer;
    DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);

    elapsedMillis timeout = 0;

    // see if we should reverse the bits in the bytes
    // this only works if in 8 bit mode.
    volatile uint32_t *flexio_shift_buffer = &_pflexio->SHIFTBUF[_fshifter];
    if (_fshifter_pins_reversed && !_fdata_4bit_mode) {
        if (_debug)
            debug.println("Using SHIFTBUFBBS");
        flexio_shift_buffer = &_pflexio->SHIFTBUFBBS[_fshifter];
    }

    // for flexIO will simply return the first buffer.
    //_dma_last_completed_frame = buffer;

    //----------------------------------------------------------------------
    // Polling FlexIO version
    //----------------------------------------------------------------------
    if (!_fuse_dma || (_dma_source == 0xff)) {
        if (_debug)
            debug.println("\tNot DMA");
        // lets try another version of this to see if cleaner.
        uint32_t *p_end = (uint32_t *)((uint8_t *)p + cb1);
        uint32_t max_time_to_wait = _timeout;
        uint32_t frame_bytes_received = 0;
        while (frame_bytes_received < frame_size_bytes) {
            while ((_pflexio->SHIFTSTAT & _fshifter_mask) == 0) {
                if (timeout > max_time_to_wait) {
                    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);

                    if (_debug)
                        debug.printf(
                            "Timeout between characters: received: %u bytes\n",
                            frame_bytes_received);
                    // wait for FlexIO shifter data
                    DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);
                    break;
                }
            }
            // Lets simplify back to single shifter for now
            uint8_t *pu8 = (uint8_t *)p;

            *p++ = *flexio_shift_buffer;
            if ((_format == pixformat_t::JPEG) && (frame_bytes_received > 0)) {
                // jpeg check for
                for (int i = 0; i < 4; i++) {
                    if ((pu8[i - 1] == 0xff) && (pu8[i] == 0xd9)) {
                        if (_debug)
                            Serial.printf("JPEG - found end marker at %u\n",
                                          frame_bytes_received + i);
                        DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);
                        return frame_bytes_received + i + 1;
                    }
                }
            }

            frame_bytes_received += sizeof(uint32_t);
            // Check to see if we filled current buffer
            if (p >= p_end) {
                // yes, now see if we have another
                if (buffer2) {
                    p = (uint32_t *)buffer2;
                    p_end = (uint32_t *)((uint8_t *)p + cb2);
                    buffer2 = nullptr;
                } else {
                    if (_debug && (frame_bytes_received < frame_size_bytes))
                        Serial.printf(
                            "Error: Image size exceeded buffer space\n");
                    break;
                }
            }
            max_time_to_wait = 50; // maybe second setting.
            timeout = 0;           // reset timeout.
        }

        DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);
        return frame_bytes_received;
    }

    //----------------------------------------------------------------------
    // Use DMA FlexIO version
    //----------------------------------------------------------------------

    _dmachannel.begin();
    _dmachannel.triggerAtHardwareEvent(_dma_source);
    active_dma_camera = this;
    _dmachannel.attachInterrupt(dmaInterruptFlexIO);

    // Some cameras or sizes we cans imply do within the DMAChannel object, so
    // lets see if that helps with the HM01B0

    // first pass split into two
    uint8_t dmas_index = 0;
    // We will do like above with both buffers, maybe later try to merge the two
    // sections.
    uint32_t cb_left = min(frame_size_bytes, cb1);
    uint8_t count_dma_settings = (cb_left / (32767 * 4)) + 1;
    uint32_t cb_per_setting = ((cb_left / count_dma_settings) + 3) &
                              0xfffffffc; // round up to next multiple of 4.

    if (_debug)
        debug.printf("frame size: %u, cb1:%u cnt dma: %u CB per: %u\n",
                     frame_size_bytes, cb1, count_dma_settings, cb_per_setting);
    if ((cb1 >= frame_size_bytes) && (count_dma_settings == 1)) {
        _dmachannel.source(*flexio_shift_buffer);
        _dmachannel.destinationBuffer((uint32_t *)buffer, frame_size_bytes);
        _dmachannel.transferSize(4);
        _dmachannel.transferCount(frame_size_bytes / 4);
        _dmachannel.disableOnCompletion();
        _dmachannel.interruptAtCompletion();
        _dmachannel.clearComplete();
#ifdef DEBUG_FLEXIO
        if (_debug) {
            Serial.printf("Camera dma channel: %u\n", _dmachannel.channel);
            dumpDMA_TCD(&_dmachannel, "CH: ");
        }
#endif
    } else {
        for (; dmas_index < count_dma_settings; dmas_index++) {
            _dmasettings[dmas_index].TCD->CSR = 0;
            _dmasettings[dmas_index].source(*flexio_shift_buffer);
            _dmasettings[dmas_index].destinationBuffer(p, cb_per_setting);
            _dmasettings[dmas_index].replaceSettingsOnCompletion(
                _dmasettings[dmas_index + 1]);
            p += (cb_per_setting / 4);
            cb_left -= cb_per_setting;
            if (cb_left < cb_per_setting)
                cb_per_setting = cb_left;
        }
        if (frame_size_bytes > cb1) {
            cb_left = frame_size_bytes - cb1;
            count_dma_settings = (cb_left / (32767 * 4)) + 1;
            cb_per_setting = ((cb_left / count_dma_settings) + 3) &
                             0xfffffffc; // round up to next multiple of 4.
            if (_debug)
                debug.printf(
                    "frame size left: %u, cb2:%u cnt dma: %u CB per: %u\n",
                    cb_left, cb2, count_dma_settings, cb_per_setting);

            p = (uint32_t *)buffer2;

            for (uint8_t i = 0; i < count_dma_settings; i++, dmas_index++) {
                _dmasettings[dmas_index].TCD->CSR = 0;
                _dmasettings[dmas_index].source(*flexio_shift_buffer);
                _dmasettings[dmas_index].destinationBuffer(p, cb_per_setting);
                _dmasettings[dmas_index].replaceSettingsOnCompletion(
                    _dmasettings[dmas_index + 1]);
                p += (cb_per_setting / 4);
                cb_left -= cb_per_setting;
                if (cb_left < cb_per_setting)
                    cb_per_setting = cb_left;
            }
        }
        dmas_index--; // lets point back to the last one
        _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[0]);
        _dmasettings[dmas_index].disableOnCompletion();
        _dmasettings[dmas_index].interruptAtCompletion();
        _dmachannel = _dmasettings[0];
        _dmachannel.clearComplete();
#ifdef DEBUG_FLEXIO
        if (_debug) {
            dumpDMA_TCD(&_dmachannel, " CH: ");
            for (uint8_t i = 0; i <= dmas_index; i++) {
                debug.printf(" %u: ", i);
                dumpDMA_TCD(&_dmasettings[i], nullptr);
            }
        }
#endif
    }

    _dma_state = DMA_STATE_ONE_FRAME;
    _pflexio->SHIFTSDEN = _fshifter_mask;
    _dmachannel.clearComplete();
    _dmachannel.clearInterrupt();
    _dmachannel.enable();

#ifdef DEBUG_FLEXIO
    if (_debug)
        debug.printf("Flexio DMA: length: %d\n", frame_size_bytes);
#endif

    timeout = 0; // reset the timeout
    // while (!_dmachannel.complete()) {
    while (_dma_state == DMA_STATE_ONE_FRAME) {
        // wait - we should not need to actually do anything during the DMA
        // transfer
        if (_dmachannel.error()) {
            debug.println("DMA error");
            if (_pflexio->SHIFTSTAT)
                debug.printf(" SHIFTSTAT %08X\n", _pflexio->SHIFTSTAT);
            debug.flush();
            uint32_t i = _pflexio->SHIFTBUF[_fshifter];
            debug.printf("Result: %x\n", i);

            _dmachannel.clearError();
            break;
        }
        if (timeout > _timeout) {
            if (_debug)
                debug.println("Timeout waiting for DMA");
            if (_pflexio->SHIFTSTAT & _fshifter_mask)
                debug.printf(" SHIFTSTAT bit was set (%08X)\n",
                             _pflexio->SHIFTSTAT);
#ifdef DEBUG_CAMERA
            debug.printf(" DMA channel #%u\n", _dmachannel.channel);
            debug.printf(" DMAMUX = %08X\n",
                         *(&DMAMUX_CHCFG0 + _dmachannel.channel));
            debug.printf(" _pflexio->SHIFTSDEN = %02X\n", _pflexio->SHIFTSDEN);
            debug.printf(" TCD CITER = %u\n", _dmachannel.TCD->CITER_ELINKNO);
            debug.printf(" TCD CSR = %08X\n", _dmachannel.TCD->CSR);
#endif
            break;
        }
    }
#ifdef USE_DEBUG_PINS
    digitalWriteFast(2, LOW);
#endif
    // arm_dcache_flush_delete(buffer, frame_size_bytes);
    if ((uint32_t)buffer >= 0x20200000u)
        arm_dcache_flush_delete(buffer, min(cb1, frame_size_bytes));
    if (frame_size_bytes > cb1) {
        if ((uint32_t)buffer2 >= 0x20200000u)
            arm_dcache_flush_delete(buffer2, frame_size_bytes - cb1);
    }

#ifdef DEBUG_FLEXIO
    if (_debug)
        dumpDMA_TCD(&_dmachannel, "CM: ");
#endif
    //    dumpDMA_TCD(&_dmasettings[0], " 0: ");
    //    dumpDMA_TCD(&_dmasettings[1], " 1: ");
    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);

    return frame_size_bytes;
}

void ImageSensor::frameStartInterruptFlexIO() {
    active_dma_camera->processFrameStartInterruptFlexIO();
}

void ImageSensor::processFrameStartInterruptFlexIO() {
#ifdef USE_DEBUG_PINS
    digitalWriteFast(5, HIGH);
#endif

    //    debug.println("VSYNC");
    // See if we read the state of it a few times if the pin stays high...
    if (digitalReadFast(_vsyncPin) && digitalReadFast(_vsyncPin) &&
        digitalReadFast(_vsyncPin) && digitalReadFast(_vsyncPin)) {
// stop this interrupt.
#ifdef USE_DEBUG_PINS
        // digitalToggleFast(2);
        digitalWriteFast(2, LOW);
        digitalWriteFast(2, HIGH);
#endif
        detachInterrupt(_vsyncPin);

        // For this pass will leave in longer DMAChain with both buffers.
        _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
        _pflexio->SHIFTERR = _fshifter_mask | _fshifter_jpeg_mask;

        _dmachannel.clearInterrupt();
        _dmachannel.clearComplete();
        _dmachannel.enable();

        DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);
        if (_format == pixformat_t::JPEG) {
            // if JPEG reenable match for ending interrupt...
            _pflexio->SHIFTEIEN |= _fshifter_jpeg_mask;
        }
    }
    asm("DSB");
#ifdef USE_DEBUG_PINS
    digitalWriteFast(5, LOW);
#endif
}

void ImageSensor::dmaInterruptFlexIO() {
    active_dma_camera->processDMAInterruptFlexIO();
}

void ImageSensor::processDMAInterruptFlexIO() {
    //    if (_debug) debug.println("PDMAI");
    _dmachannel.clearInterrupt();
#ifdef USE_DEBUG_PINS
    //  digitalToggleFast(2);
    digitalWriteFast(2, HIGH);
    digitalWriteFast(2, LOW);
#endif
    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);

    if (_dma_state == DMA_STATE_ONE_FRAME) {
        _dma_state = DMA_STATE_STOPPED;
        asm("DSB");
        return;

    } else if (_dma_state == DMASTATE_STOP_REQUESTED) {
        _dmachannel.disable();
        _frame_buffer_1 = nullptr;
        _frame_buffer_2 = nullptr;
        _callback = nullptr;
        _dma_state = DMA_STATE_STOPPED;
        asm("DSB");
        return;
    }

#if 0
  static uint8_t debug_print_count = 8;
  if (debug_print_count) {
    debug_print_count--;
    debug.printf("PDMAIF: %x\n", (uint32_t)_dmachannel.TCD->DADDR);
    dumpDMA_TCD(&_dmachannel," CH: ");

  }
#endif
    _dmachannel.clearComplete();
    const uint32_t frame_size_bytes = _width * _height * _bytesPerPixel;
    if (((uint32_t)_dmachannel.TCD->DADDR) == (uint32_t)_frame_buffer_1) {
        _dma_last_completed_frame = _frame_buffer_2;
        if ((uint32_t)_frame_buffer_2 >= 0x20200000u)
            arm_dcache_flush_delete(_frame_buffer_2,
                              min(_frame_buffer_2_size, frame_size_bytes));
    } else {
        _dma_last_completed_frame = _frame_buffer_1;
        if ((uint32_t)_frame_buffer_1 >= 0x20200000u)
            arm_dcache_flush_delete(_frame_buffer_1,
                              min(_frame_buffer_1_size, frame_size_bytes));
    }

    _dma_last_completed_image_size = frame_size_bytes;

    if (_callback)
        (*_callback)(_dma_last_completed_frame); // TODO: use EventResponder
    // if we disabled the DMA, then setup to wait for vsyncpin...
    if ((_dma_last_completed_frame == _frame_buffer_2) ||
        (_frame_buffer_1_size >= frame_size_bytes)) {
        _dma_active = false;
        // start up interrupt to look for next start of interrupt.

        if (_dma_state == DMASTATE_RUNNING)
            attachInterrupt(_vsyncPin, &frameStartInterruptFlexIO, RISING);
    }

    asm("DSB");
}

//=============================================================================
// Flexio - Start a read.
//=============================================================================
bool ImageSensor::startReadFlexIO(bool (*callback)(void *frame_buffer),
                                  void *fb1, size_t cb1, void *fb2,
                                  size_t cb2) {

    // Make sure DMA is avaialble on this Flexio object
    if (_dma_source == 0xff)
        return false; // nope

    const uint32_t frame_size_bytes = _width * _height * _bytesPerPixel;
    //-------------------------------------------------------------------------
    // Experiment if we are doing JPEG - see if we can setup another shifter
    // in compare mode to look for a end of frame marker.
    //-------------------------------------------------------------------------
    if (_format == pixformat_t::JPEG) {
        if (_debug)
            debug.printf("startReadFlexIO called in JPEG mode: fshifter:%u\n", _fshifter_jpeg);
        if (_fshifter_jpeg == 0xff) {
            DBGdigitalWriteFast(3, HIGH);
            delayMicroseconds(10);
            DBGdigitalWriteFast(3, LOW);
            delayMicroseconds(10);
            DBGdigitalWriteFast(3, HIGH);
            delayMicroseconds(10);
            DBGdigitalWriteFast(3, LOW);
            delayMicroseconds(10);
            DBGdigitalWriteFast(3, HIGH);
            delayMicroseconds(10);
            DBGdigitalWriteFast(3, LOW);
            if (!_pflex->claimShifter(_fshifter - 1))
                return false; // could not allocate shifter
            _fshifter_jpeg = _fshifter - 1;
            _fshifter_jpeg_mask = (1 << _fshifter_jpeg);

            // Lets add our object to the
            _pflex->addIOHandlerCallback(this);

            // We need to setup this shifter to be a compare type.
            // 16 bits and use the shifter above us
            _pflexio->SHIFTCFG[_fshifter_jpeg] = FLEXIO_SHIFTCFG_PWIDTH(31) | FLEXIO_SHIFTCFG_INSRC;
            // match continuous mode
            _pflexio->SHIFTCTL[_fshifter_jpeg] = FLEXIO_SHIFTCTL_TIMSEL(_ftimer) | FLEXIO_SHIFTCTL_SMOD(0x5);
            // Setup the match value.
            //_pflexio->SHIFTBUF[_fshifter_jpeg] = 0xd90000FF; // looking for a -0xff 0xd9 with masks of 0...
            _pflexio->SHIFTBUF[_fshifter_jpeg] = 0xd9ff0000; // looking for a -0xff 0xd9 with masks of 0...
            //_pflexio->SHIFTBUF[_fshifter_jpeg] = 0xffd90000; // looking for a -0xff 0xd9 with masks of 0...
        }
        // enable the error interrupt on this channel - make sure it's state is not set...
        _pflexio->SHIFTERR = _fshifter_mask;
        _pflexio->SHIFTEIEN |= _fshifter_jpeg_mask;
#ifdef DEBUG_FLEXIO
        debug.printf("FLEXIO: JPEG Shifter:%u\n", _fshifter_jpeg);
        debug.printf("     SHIFTCFG =  %08X\n", _pflexio->SHIFTCFG[_fshifter_jpeg]);
        debug.printf("     SHIFTCTL =  %08X\n", _pflexio->SHIFTCTL[_fshifter_jpeg]);
        debug.printf("     SHIFTBUF =  %08X\n", _pflexio->SHIFTBUF[_fshifter_jpeg]);
        debug.printf("     SHIFTBUF =  %08X\n", _pflexio->SHIFTBUF[_fshifter_jpeg]);
        debug.printf("     SHIFTSTAT =  %08X\n", _pflexio->SHIFTSTAT);
        debug.printf("     SHIFTSIEN =  %08X\n", _pflexio->SHIFTSIEN);
        debug.printf("     SHIFTERR =  %08X\n", _pflexio->SHIFTERR);
        debug.printf("     SHIFTEIEN =  %08X\n", _pflexio->SHIFTEIEN);

#endif
    }

    // lets handle a few cases.
    if (fb1 == nullptr)
        return false;
    if (cb1 < frame_size_bytes) {
        if ((fb2 == nullptr) || ((cb1 + cb2) < frame_size_bytes))
            return false;
    }

    _frame_buffer_1 = (uint8_t *)fb1;
    _frame_buffer_1_size = cb1;
    _frame_buffer_2 = (uint8_t *)fb2;
    _frame_buffer_2_size = cb2;
    _callback = callback;
    active_dma_camera = this;

    // flexio_configure(); // one-time hardware setup
    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;
    uint32_t *p = (uint32_t *)fb1;

    //----------------------------------------------------------------------
    // Use DMA FlexIO version
    //----------------------------------------------------------------------
    // Currently lets setup for only one shifter
    //    digitalWriteFast(2, HIGH);

    _dmachannel.begin();
    _dmachannel.triggerAtHardwareEvent(_dma_source);
    active_dma_camera = this;
    _dmachannel.attachInterrupt(dmaInterruptFlexIO);

    // Two versions.  If one buffer is large enough, we will use the two buffers
    // to read in two frames.  If the combined in large enough for one frame, we
    // will setup to read one frame but still interrupt on each buffer filled
    // completion

    uint8_t dmas_index = 0;

    uint32_t cb_left = min(frame_size_bytes, cb1);
    uint8_t count_dma_settings = (cb_left / (32767 * 4)) + 1;
    uint32_t cb_per_setting = ((cb_left / count_dma_settings) + 3) &
                              0xfffffffc; // round up to next multiple of 4.
    if (_debug)
        debug.printf("frame size: %u, cb1:%u cnt dma: %u CB per: %u\n",
                     frame_size_bytes, cb1, count_dma_settings, cb_per_setting);

    for (; dmas_index < count_dma_settings; dmas_index++) {
        _dmasettings[dmas_index].TCD->CSR = 0;
        _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
        _dmasettings[dmas_index].destinationBuffer(p, cb_per_setting);
        _dmasettings[dmas_index].replaceSettingsOnCompletion(
            _dmasettings[dmas_index + 1]);
        p += (cb_per_setting / 4);
        cb_left -= cb_per_setting;
        if (cb_left < cb_per_setting)
            cb_per_setting = cb_left;
    }
    // Interrupt after each buffer is filled.
    _dmasettings[dmas_index - 1].interruptAtCompletion();
    if (cb1 >= frame_size_bytes) {
        _dmasettings[dmas_index - 1].disableOnCompletion(); // full frame
        if (fb2 && (cb2 >= frame_size_bytes))
            cb_left = min(frame_size_bytes, cb2);
        else
            cb_left = 0; // We don't have second buffer to use.
    } else
        cb_left =
            frame_size_bytes - cb1; // need second buffer to complete one frame.

    if (cb_left) {
        count_dma_settings = (cb_left / (32767 * 4)) + 1;
        cb_per_setting = ((cb_left / count_dma_settings) + 3) &
                         0xfffffffc; // round up to next multiple of 4.
        if (_debug)
            debug.printf("frame size left: %u, cb2:%u cnt dma: %u CB per: %u\n",
                         cb_left, cb2, count_dma_settings, cb_per_setting);

        p = (uint32_t *)fb2;

        for (uint8_t i = 0; i < count_dma_settings; i++, dmas_index++) {
            _dmasettings[dmas_index].TCD->CSR = 0;
            _dmasettings[dmas_index].source(_pflexio->SHIFTBUF[_fshifter]);
            _dmasettings[dmas_index].destinationBuffer(p, cb_per_setting);
            _dmasettings[dmas_index].replaceSettingsOnCompletion(
                _dmasettings[dmas_index + 1]);
            p += (cb_per_setting / 4);
            cb_left -= cb_per_setting;
            if (cb_left < cb_per_setting)
                cb_per_setting = cb_left;
        }
        _dmasettings[dmas_index - 1].disableOnCompletion();
        _dmasettings[dmas_index - 1].interruptAtCompletion();
    }
    dmas_index--; // lets point back to the last one
    _dmasettings[dmas_index].replaceSettingsOnCompletion(_dmasettings[0]);
    _dmachannel = _dmasettings[0];
    _dmachannel.clearComplete();

#ifdef DEBUG_FLEXIO
    if (_debug) {
        dumpDMA_TCD(&_dmachannel, " CH: ");
        for (uint8_t i = 0; i <= dmas_index; i++) {
            debug.printf(" %u: ", i);
            dumpDMA_TCD(&_dmasettings[i], nullptr);
        }
    }
    debug.printf("Flexio DMA: length: %d\n", frame_size_bytes);

#endif

    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;

    _dma_last_completed_frame = nullptr;
    _dma_frame_count = 0;

    _dma_state = DMASTATE_RUNNING;

    //-------------------------------------------------------------------------
    // Lets use interrupt on interrupt on VSYNC pin to start the capture of a
    // frame
    _dma_active = false;
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 102);
    // NVIC_SET_PRIORITY(dma_flexio.channel & 0xf, 102);
    attachInterrupt(_vsyncPin, &frameStartInterruptFlexIO, RISING);
    _pflexio->SHIFTSDEN = _fshifter_mask;

    return true;
}

bool ImageSensor::stopReadFlexIO() {
    // first disable the vsync interrupt
    detachInterrupt(_vsyncPin);
    if (!_dma_active) {
        _dma_state = DMA_STATE_STOPPED;
    } else {
        cli();
        if (_dma_state != DMA_STATE_STOPPED)
            _dma_state = DMASTATE_STOP_REQUESTED;
        sei();
    }
    return true;
}

void ImageSensor::dumpDMA_TCD(DMABaseClass *dmabc, const char *psz_title) {
    if (psz_title)
        debug.print(psz_title);
    debug.printf("%x %x: ", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

    debug.printf(
        "SA:%x SO:%d AT:%x (SM:%x SS:%x DM:%x DS:%x) NB:%x SL:%d DA:%x "
        "DO: %d CI:%x DL:%x CS:%x BI:%x\n",
        (uint32_t)dmabc->TCD->SADDR, dmabc->TCD->SOFF, dmabc->TCD->ATTR,
        (dmabc->TCD->ATTR >> 11) & 0x1f, (dmabc->TCD->ATTR >> 8) & 0x7,
        (dmabc->TCD->ATTR >> 3) & 0x1f, (dmabc->TCD->ATTR >> 0) & 0x7,
        dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR,
        dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA,
        dmabc->TCD->CSR, dmabc->TCD->BITER);
}

//=============================================================================
// Callback function for flexio
//=============================================================================
bool ImageSensor::call_back(FlexIOHandler *pflex) {
    DBGdigitalWriteFast(3, HIGH);
    IMXRT_FLEXIO_t *p = &pflex->port();
    // if neither of them are active return quick...
    // hack may want to count interrupts to get the last data
    static uint8_t count_SHIFSIEN_ints = 0;

    if ((p->SHIFTEIEN == 0) && (p->SHIFTSIEN == 0))
        return false;

    if (_debug && ((p->SHIFTSTAT) || (p->SHIFTERR)))
        debug.printf("CB: %x %x\n", p->SHIFTSTAT, p->SHIFTERR);

    // First check for the Shifter match for the ending 0xFF 0xD9
    if ((p->SHIFTEIEN & _fshifter_jpeg_mask) && (p->SHIFTERR & _fshifter_jpeg_mask)) {
        // we have a match disable us.  Maybe enable on main shifter as to allow
        // last stuff to be saved away.
        p->SHIFTERR = _fshifter_jpeg_mask;
        p->SHIFTSTAT = _fshifter_jpeg_mask;
        p->SHIFTEIEN &= ~_fshifter_jpeg_mask;
        p->SHIFTSIEN |= _fshifter_mask; // enable interrupt on this one...
        count_SHIFSIEN_ints = 2;        // lets try receiving two

        // I think the DMA will have already processed the SHIFT state, so only test we have it enable
    } else if ((p->SHIFTSIEN & _fshifter_mask) /* && (p->SHIFTSTAT & _fshifter_mask)*/) {
        count_SHIFSIEN_ints--;
        if (count_SHIFSIEN_ints == 0) {
            // So we have shifter output after we received the 0xff 0xd9
            // now to disable the DMA operation and signal the complete frame.
            // Not sure if I need to wait for one more transfer after this or not
            // what happens first the DMA operation or the Interrupt call?
            p->SHIFTSIEN &= ~_fshifter_mask; // turn off this interrupt.

            // Now lets try to act like the dma completion interrupt.
            _dmachannel.disable();

            // going to assume one frame buffer used.
            uint32_t end_address = (uint32_t)_dmachannel.TCD->DADDR;
            uint32_t start_address = 0;
            uint32_t frame_size_bytes = 0xffffffff;
            if (end_address >= (uint32_t)_frame_buffer_1) {
                start_address = (uint32_t)_frame_buffer_1;
                frame_size_bytes = end_address - start_address;
            }

            if (end_address >= (uint32_t)_frame_buffer_2) {
                uint32_t frame_size_bytes2 = end_address - (uint32_t)_frame_buffer_2;
                if (frame_size_bytes2 < frame_size_bytes) {
                    start_address = (uint32_t)_frame_buffer_2;
                    frame_size_bytes = frame_size_bytes2;
                }
            }
            _dma_last_completed_frame = (uint8_t *)start_address;
            _dma_last_completed_image_size = frame_size_bytes;

            if (start_address >= 0x20200000u)
                arm_dcache_flush_delete((void *)start_address, frame_size_bytes);

            if (_callback)
                (*_callback)(_dma_last_completed_frame);

            // bugbug: Lets reset to first dma setting, as we are probably in the middle of some other
            // one...
            _dmachannel = _dmasettings[0];

            if (_dma_state == DMASTATE_RUNNING)
                attachInterrupt(_vsyncPin, &frameStartInterruptFlexIO, RISING);
        }
    }

    DBGdigitalWriteFast(3, LOW);
    asm("dsb");   // not sure if we need this here or not
    return false; // right now always return false...
}

//=============================================================================
// hardware_configure() - will check the pins to see if the camera is configured
// to hse CSI and if not check for FlexIO
//=============================================================================
bool ImageSensor::hardware_configure() {
    // first try CSI
    if (_debug)
        debug.println("hardware_configure - try CSI");
    if (!csi_configure()) {
        if (_debug)
            debug.println("\tfailed try FLEXIO");
        if (!flexio_configure()) {
            // Later we can expand this to look at GPIO...
            if (_debug)
                debug.println("\tfailed");
            return false;
        }
    }
    return true;
}

// Default function to set the VSYNC priority
// only needed in some implementations.
void ImageSensor::setVSyncISRPriority(uint8_t priority) {
    if (_debug)
        debug.printf("\n@@@ SetVSYNC ISR priority: %u camera Input:%u\n", priority, _cameraInput);

    if (_cameraInput != CAMERA_INPUT_CSI) {
        NVIC_SET_PRIORITY(IRQ_GPIO6789, priority);
    }
}

struct pwm_pin_info_struct {
    uint8_t type;    // 0=no pwm, 1=flexpwm, 2=quad
    uint8_t module;  // 0-3, 0-3
    uint8_t channel; // 0=X, 1=A, 2=B
    uint8_t muxval;  //
};
extern const struct pwm_pin_info_struct pwm_pin_info[];

bool checkIfPinSupportsPWM(uint8_t pin) {
    if (pin >= CORE_NUM_DIGITAL)
        return false;
    return (pwm_pin_info[pin].type != 0);
}

//=============================================================================
// Begin the XCLK - this also needs to do quick test to see if we are running
// CSI pins or not...
//
// If xclk is CSI we will use that by default.
//=============================================================================
void ImageSensor::beginXClk() {
    // Generates 8 MHz signal using PWM... Will speed up.

    if (verifyCSIPin(_xclkPin, CSI_MCLK)) {
        if (_debug)
            debug.println("** beginXCLK CSI **");
        configureCSIPin(_xclkPin);

        CCM_CCGR2 &= ~CCM_CCGR2_CSI(CCM_CCGR_ON); // turn off csi clock
        if (_xclk_freq <= 16) {
            _xclk_freq = 12;
            CCM_CSCDR3 = CCM_CSCDR3_CSI_CLK_SEL(0) | CCM_CSCDR3_CSI_PODF(1); // set csi clock source and divide by 2 = 12 MHz
            //} else if (_xclk_freq <= 16) {
            //    _xclk_freq = 15;
            //    CCM_CSCDR3 = CCM_CSCDR3_CSI_CLK_SEL(2) | CCM_CSCDR3_CSI_PODF(7);  // 120/8 = 15 MHz
        } else if (_xclk_freq <= 20) {
            _xclk_freq = 20;
            CCM_CSCDR3 = CCM_CSCDR3_CSI_CLK_SEL(2) | CCM_CSCDR3_CSI_PODF(5); // 120/6 = 20 MHz
        } else {
            // for now lets try up to 24mhz
            _xclk_freq = 24;
            CCM_CSCDR3 = CCM_CSCDR3_CSI_CLK_SEL(2) | CCM_CSCDR3_CSI_PODF(4); // 120/6 = 20 MHz
        }

        IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 0b100; // alt4
        IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_05 = 0x18U; // 50MHz speed, DSE_3

        CCM_CCGR2 |= CCM_CCGR2_CSI(CCM_CCGR_ON);
    } else {
        // USE PWM
        if (_debug)
            debug.println("** beginXCLK PWM **");
        analogWriteFrequency(_xclkPin, _xclk_freq * 1000000);
        analogWrite(_xclkPin, 127);
    }
    delay(100); // 9mhz works, but try to reduce to debug timings with logic
                // analyzer
}

void ImageSensor::endXClk() {
#if defined(__IMXRT1062__) // Teensy 4.x
    if (_cameraInput == CAMERA_INPUT_CSI) {
        CCM_CCGR2 &= ~CCM_CCGR2_CSI(CCM_CCGR_ON);
    } else {
        analogWrite(_xclkPin, 0);
    }
#else
    NRF_I2S->TASKS_STOP = 1;
#endif
}

//=============================================================================
// this is the main code to configure for FlexIO
//=============================================================================
bool ImageSensor::flexio_configure() {
#ifdef DEBUG_CAMERA
    debug.println("ImageSensor::flexio_configure() called");
#endif
    uint8_t tpclk_pin;
    _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_pclkPin, tpclk_pin);
    if (!_pflex) {
        debug.printf("ImageSensor PCLK(%u) is not a valid Flex IO pin\n",
                     _pclkPin);
        return false;
    }
    _pflexio = &(_pflex->port());

    // Quick and dirty:
    uint8_t thsync_pin = _pflex->mapIOPinToFlexPin(_hrefPin);
    uint8_t tg0 = _pflex->mapIOPinToFlexPin(_dPins[0]);
    uint8_t tg1 = _pflex->mapIOPinToFlexPin(_dPins[1]);
    uint8_t tg2 = _pflex->mapIOPinToFlexPin(_dPins[2]);
    uint8_t tg3 = _pflex->mapIOPinToFlexPin(_dPins[3]);
    uint8_t shiftctl_pinctl = tg0; // most cases it will be the first pin
    // make sure the minimum here is valid:
    if ((thsync_pin == 0xff) || (tg0 == 0xff) || (tg1 == 0xff) ||
        (tg2 == 0xff) || (tg3 == 0xff)) {
        debug.printf("OV767X Some pins did not map to valid Flex IO pin\n");
        if (_debug)
            debug.printf(
                "    HSYNC(%u %u) G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)",
                _hrefPin, thsync_pin, _dPins[0], tg0, _dPins[1], tg1, _dPins[2],
                tg2, _dPins[3], tg3);
        return false;
    }
    // Verify that the G numbers are consecutive... Should use arrays!
    if ((tg1 != (tg0 + 1)) || (tg2 != (tg0 + 2)) || (tg3 != (tg0 + 3))) {
        debug.printf("OV767X Flex IO pins G0-G3 are not consective\n");
        if (_debug)
            debug.printf("    G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)",
                         _dPins[0], tg0, _dPins[1], tg1, _dPins[2], tg2,
                         _dPins[3], tg3);

        if ((tg1 == (tg0 - 1)) && (tg2 == (tg0 - 2)) && (tg3 == (tg0 - 3))) {
            if (_debug)
                debug.println("\tpins reversed");
            _fshifter_pins_reversed = true;
            shiftctl_pinctl = tg3; // but reversed so use this in 4 bit mode
        } else
            return false;
    }
    // see if the caller set us explicitly into 4 bit mode
    if (!_fdata_4bit_mode && _dPins[4] != 0xff) {
        uint8_t tg4 = _pflex->mapIOPinToFlexPin(_dPins[4]);
        uint8_t tg5 = _pflex->mapIOPinToFlexPin(_dPins[5]);
        uint8_t tg6 = _pflex->mapIOPinToFlexPin(_dPins[6]);
        uint8_t tg7 = _pflex->mapIOPinToFlexPin(_dPins[7]);
        if (!_fshifter_pins_reversed) {
            if ((tg4 != (tg0 + 4)) || (tg5 != (tg0 + 5)) || (tg6 != (tg0 + 6)) ||
                (tg7 != (tg0 + 7))) {
                debug.printf(
                    "OV767X Flex IO pins G4-G7 are not consective with G0-3\n");
                if (_debug)
                    debug.printf("    G0(%u %u) G4(%u %u) G5(%u %u) G6(%u %u) G7(%u %u)",
                                 _dPins[0], tg0, _dPins[4], tg4, _dPins[5], tg5, _dPins[6], tg6, _dPins[7], tg7);
                return false;
            }
        } else {
            if ((tg4 != (tg0 - 4)) || (tg5 != (tg0 - 5)) || (tg6 != (tg0 - 6)) || (tg7 != (tg0 - 7))) {
                debug.printf(
                    "OV767X Flex IO pins G4-G7 are not consective with G0-3\n");
                if (_debug)
                    debug.printf("    G0(%u %u) G4(%u %u) G5(%u %u) G6(%u %u) G7(%u %u)",
                                 _dPins[0], tg0, _dPins[4], tg4, _dPins[5], tg5, _dPins[6],
                                 tg6, _dPins[7], tg7);
                return false;
            }
            shiftctl_pinctl = tg7; // but reversed so use this in 8 bit mode
        }
        if (_debug)
            debug.println("Custom - Flexio is 8 bit mode");
    } else {
        // Fiexio in 4 bit mode.
        _hw_config = TEENSY_MICROMOD_FLEXIO_4BIT;
        if (_debug)
            debug.println("Custom - Flexio is 4 bit mode");
    }
    // Needs Shifter 3 (maybe 7 would work as well?)
    // make sure we handle if we already claimed it before.
    if ((_fshifter != 3) && (_fshifter != 7)) {
        if (_pflex->claimShifter(3))
            _fshifter = 3;
        else if (_pflex->claimShifter(7))
            _fshifter = 7;
        else {
            if (_debug)
                debug.printf("OV767X Flex IO: Could not claim Shifter 3 or 7\n");
            return false;
        }
    }
    _fshifter_mask = 1 << _fshifter;                     // 4 channels.
    _dma_source = _pflex->shiftersDMAChannel(_fshifter); // looks like they use

    // Now request one timer
    uint8_t _ftimer = _pflex->requestTimers(); // request 1 timer.
    if (_ftimer == 0xff) {
        if (_debug)
            debug.printf("ImageSensor Flex IO: failed to request timer\n");
        return false;
    }

    _pflex->setIOPinToFlexMode(_hrefPin);
    _pflex->setIOPinToFlexMode(_pclkPin);
    _pflex->setIOPinToFlexMode(_dPins[0]);
    _pflex->setIOPinToFlexMode(_dPins[1]);
    _pflex->setIOPinToFlexMode(_dPins[2]);
    _pflex->setIOPinToFlexMode(_dPins[3]);
    if (!_fdata_4bit_mode && (_dPins[4] != 0xff)) {
        _pflex->setIOPinToFlexMode(_dPins[4]);
        _pflex->setIOPinToFlexMode(_dPins[5]);
        _pflex->setIOPinToFlexMode(_dPins[6]);
        _pflex->setIOPinToFlexMode(_dPins[7]);
    }

    // We already configured the clock to allow access.
    // Now sure yet aoub configuring the actual colock speed...

    /*
        CCM_CSCMR2 |= CCM_CSCMR2__pflex->CLK_SEL(3); // 480 MHz from USB PLL

        CCM_CS1CDR = (CCM_CS1CDR
            & ~(CCM_CS1CDR__pflex->CLK_PRED(7) |
       CCM_CS1CDR__pflex->CLK_PODF(7))) | CCM_CS1CDR__pflex->CLK_PRED(1) |
       CCM_CS1CDR__pflex->CLK_PODF(1);


        CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);
    */
    // clksel(0-3PLL4, Pll3 PFD2 PLL5, *PLL3_sw)
    // clk_pred(0, 1, 2, 7) - divide (n+1)
    // clk_podf(0, *7) divide (n+1)
    // So default is 480mhz/16
    // Clock select, pred, podf:
    _pflex->setClockSettings(3, 1, 1);

#ifdef DEBUG_FLEXIO
    debug.println("FlexIO Configure");
    debug.printf(" CCM_CSCMR2 = %08X\n", CCM_CSCMR2);
    uint32_t div1 = ((CCM_CS1CDR >> 9) & 7) + 1;
    uint32_t div2 = ((CCM_CS1CDR >> 25) & 7) + 1;
    debug.printf(" div1 = %u, div2 = %u\n", div1, div2);
    debug.printf(" FlexIO Frequency = %.2f MHz\n",
                 480.0 / (float)div1 / (float)div2);
    debug.printf(" CCM_CCGR3 = %08X\n", CCM_CCGR3);
    debug.printf(" FlexIO CTRL = %08X\n", _pflexio->CTRL);
    debug.printf(" FlexIO Config, param=%08X\n", _pflexio->PARAM);
#endif

#define FLEXIO_TIMER_TRIGGER_SEL_PININPUT(x) ((uint32_t)(x) << 1U)
    if (_hw_config == TEENSY_MICROMOD_FLEXIO_8BIT) {
#ifdef DEBUG_FLEXIO
        debug.println("8Bit FlexIO");
#endif
        // SHIFTCFG, page 2927
        //  PWIDTH: number of bits to be shifted on each Shift clock
        //          0 = 1 bit, 1-3 = 4 bit, 4-7 = 8 bit, 8-15 = 16 bit, 16-31 =
        //          32 bit
        //  INSRC: Input Source, 0 = pin, 1 = Shifter N+1 Output
        //  SSTOP: Stop bit, 0 = disabled, 1 = match, 2 = use zero, 3 = use one
        //  SSTART: Start bit, 0 = disabled, 1 = disabled, 2 = use zero, 3 = use
        //  one
        // setup the for shifters
        _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(7);

        // Timer model, pages 2891-2893
        // TIMCMP, page 2937
        // using 1 shifters
        _pflexio->TIMCMP[_ftimer] = (8U * 1) - 1;

        // TIMCTL, page 2933
        //  TRGSEL: Trigger Select ....
        //          4*N - Pin 2*N input
        //          4*N+1 - Shifter N status flag
        //          4*N+2 - Pin 2*N+1 input
        //          4*N+3 - Timer N trigger output
        //  TRGPOL: 0 = active high, 1 = active low
        //  TRGSRC: 0 = external, 1 = internal
        //  PINCFG: timer pin, 0 = disable, 1 = open drain, 2 = bidir, 3 =
        //  output PINSEL: which pin is used by the Timer input or output
        //  PINPOL: 0 = active high, 1 = active low
        //  TIMOD: mode, 0 = disable, 1 = 8 bit baud rate, 2 = 8 bit PWM, 3 = 16
        //  bit
        _pflexio->TIMCTL[_ftimer] =
            FLEXIO_TIMCTL_TIMOD(3) |
            FLEXIO_TIMCTL_PINSEL(tpclk_pin) // "Pin" is 16 = PCLK
            //| FLEXIO_TIMCTL_TRGSEL(4 * (thsync_pin/2)) // "Trigger" is 12 =
            // HSYNC
            | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(
                  thsync_pin)) // "Trigger" is 12 = HSYNC
            | FLEXIO_TIMCTL_TRGSRC;
    } else {

#ifdef DEBUG_FLEXIO
        debug.println("4Bit FlexIO");
#endif
        if (!supports4BitMode()) {
            debug.println("Error: Camera does not support 4 FlexIO pin mode");
            return false;
        }
        // registers fields shown in 8 bit mode.
        _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(3); // 4 pins
        _pflexio->TIMCMP[_ftimer] = 15;
        _pflexio->TIMCTL[_ftimer] =
            FLEXIO_TIMCTL_TIMOD(3) |
            FLEXIO_TIMCTL_PINSEL(tpclk_pin) // "Pin" is 16 = PCLK
            | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(
                  thsync_pin)) // "Trigger" is 12 = HSYNC
            | FLEXIO_TIMCTL_TRGSRC;
    }

#ifdef DEBUG_FLEXIO
    debug.printf("TIMCTL: %08X PINSEL: %x THSYNC: %x\n",
                 _pflexio->TIMCTL[_ftimer], tpclk_pin, thsync_pin);
#endif

    // SHIFTCTL, page 2926
    //  TIMSEL: which Timer is used for controlling the logic/shift register
    //  TIMPOL: 0 = shift of positive edge, 1 = shift on negative edge
    //  PINCFG: 0 = output disabled, 1 = open drain, 2 = bidir, 3 = output
    //  PINSEL: which pin is used by the Shifter input or output
    //  PINPOL: 0 = active high, 1 = active low
    //  SMOD: 0 = disable, 1 = receive, 2 = transmit, 4 = match store,
    //        5 = match continuous, 6 = state machine, 7 = logic
    // 4 shifters
    _pflexio->SHIFTCTL[_fshifter] = FLEXIO_SHIFTCTL_TIMSEL(_ftimer) |
                                    FLEXIO_SHIFTCTL_SMOD(1) |
                                    FLEXIO_SHIFTCTL_PINSEL(shiftctl_pinctl);

    // TIMCFG, page 2935
    //  TIMOUT: Output
    //          0 = output is logic one when enabled and is not affected by
    //          timer reset 1 = output is logic zero when enabled and is not
    //          affected by timer reset 2 = output is logic one when enabled and
    //          on timer reset 3 = output is logic zero when enabled and on
    //          timer reset
    //  TIMDEC: Decrement
    //          0 = on FlexIO clock, Shift clock equals Timer output
    //          1 = on Trigger input (both edges), Shift clock equals Timer
    //          output 2 = on Pin input (both edges), Shift clock equals Pin
    //          input 3 = on Trigger input (both edges), Shift clock equals
    //          Trigger input
    //  TIMRST: Reset
    //          0 = never reset
    //          2 = on Timer Pin equal to Timer Output
    //          3 = on Timer Trigger equal to Timer Output
    //          4 = on Timer Pin rising edge
    //          6 = on Trigger rising edge
    //          7 = on Trigger rising or falling edge
    //  TIMDIS: Disable
    //          0 = never disabled
    //          1 = disabled on Timer N-1 disable
    //          2 = disabled on Timer compare
    //          3 = on Timer compare and Trigger Low
    //          4 = on Pin rising or falling edge
    //          5 = on Pin rising or falling edge provided Trigger is high
    //          6 = on Trigger falling edge
    //  TIMENA
    //          0 = always enabled
    //          1 = enabled on Timer N-1 enable
    //          2 = enabled on Trigger high
    //          3 = enabled on Trigger high and Pin high
    //          4 = enabled on Pin rising edge
    //          5 = enabled on Pin rising edge and Trigger high
    //          6 = enabled on Trigger rising edge
    //          7 = enabled on Trigger rising or falling edge
    //  TSTOP Stop bit, 0 = disabled, 1 = on compare, 2 = on disable, 3 = on
    //  either TSTART: Start bit, 0 = disabled, 1 = enabled
    _pflexio->TIMCFG[_ftimer] =
        FLEXIO_TIMCFG_TIMOUT(1) | FLEXIO_TIMCFG_TIMDEC(2) |
        FLEXIO_TIMCFG_TIMENA(6) | FLEXIO_TIMCFG_TIMDIS(6 /*tried 3*/);

    // CTRL, page 2916
    _pflexio->CTRL = FLEXIO_CTRL_FLEXEN; // enable after everything configured

#ifdef DEBUG_FLEXIO
    debug.printf(" FLEXIO:%u Shifter:%u Timer:%u\n", _pflex->FlexIOIndex(),
                 _fshifter, _ftimer);
    debug.print("     SHIFTCFG = ");
    debug.printf(" %08X", _pflexio->SHIFTCFG[_fshifter]);
    debug.print("\n     SHIFTCTL = ");
    debug.printf(" %08X", _pflexio->SHIFTCTL[_fshifter]);
    debug.printf("\n     TIMCMP = %08X\n", _pflexio->TIMCMP[_ftimer]);
    debug.printf("     TIMCFG = %08X\n", _pflexio->TIMCFG[_ftimer]);
    debug.printf("     TIMCTL = %08X\n", _pflexio->TIMCTL[_ftimer]);
#endif
    _cameraInput = CAMERA_INPUT_FLEXIO;
    return true;
}

//=============================================================================
// Reading using CSI support
//=============================================================================
bool ImageSensor::checkForCSIPins() {
    if (_fdata_4bit_mode)
        return false; // currently we don't support 4 bit mode
    if (!verifyCSIPin(_vsyncPin, CSI_VS))
        return false;
    if (!verifyCSIPin(_hrefPin, CSI_HS))
        return false;
    if (!verifyCSIPin(_pclkPin, CSI_PCLK))
        return false;
    if (!verifyCSIPin(_xclkPin, CSI_MCLK))
        return false;

    // note we are wanting 4 or 8 data pins but they start at 2 in this case
    for (uint8_t i = 0; i < 8; i++)
        if (!verifyCSIPin(_dPins[i], i + 2))
            return false;

    CCM_CCGR2 |= CCM_CCGR2_CSI(CCM_CCGR_ON); // turn on CSI clocks

    return true;
}

bool ImageSensor::csi_configure() {
    // lets first verify that all of the CSI pins are valid for CSI
    if (!checkForCSIPins())
        return false;
    // Lets configure all of the IO pins to CSI
    // Need to see what we do with XCLK
    configureCSIPin(_vsyncPin);
    configureCSIPin(_hrefPin);
    configureCSIPin(_pclkPin);

    for (uint8_t i = 0; i < 8; i++)
        configureCSIPin(_dPins[i]);
    // complete the setup with the reset dma method.
    return csi_reset_dma();
}

bool ImageSensor::csi_reset_dma() {
    // Update the XCLK

    // Done earlier
    // configureCSIPin(_xclkPin);
    // lets initialize the CSI
    NVIC_DISABLE_IRQ(IRQ_CSI);
    // set initial values in CSICR1
    CSI_CSICR1 |=
        (CSI_CSICR1_SOF_POL |   // 1 for  VSYNC rising edge
         CSI_CSICR1_FCC |       // FCC_MASK
         CSI_CSICR1_HSYNC_POL | // HSYNC_POL 1 for active high
         CSI_CSICR1_GCLK_MODE | // 1= GCLK_MODE PCLK only valid when HSYNC high
         CSI_CSICR1_REDGE);     // Use pixel clock rising edge

    // Set Image Parameters--width in bytes and height
    CSI_CSIIMAG_PARA = CSI_CSIIMAG_PARA_IMAGE_WIDTH(_bytesPerPixel * _width) |
                       CSI_CSIIMAG_PARA_IMAGE_HEIGHT(_height);

    CSI_CSICR3 = CSI_CSICR3_FRMCNT_RST |            // clear the frame counter
                 CSI_CSICR3_DMA_REQ_EN_RFF;         //  Enable RxFIFO DMA request
    CSI_CSICR2 |= CSI_CSICR2_DMA_BURST_TYPE_RFF(3); // RxFIFO inc by 16

    // Start off only write to memory if CSI is enabled. Option 0
    CSI_CSICR18 = (CSI_CSICR18 & ~((CSI_CSICR18_MASK_OPTION(3)) |
                                   CSI_CSICR18_MASK_OPTION(0)));

    // setup interrupts on DMA done on either buffer and maybe Start of frame.
    CSI_CSICR1 |= CSI_CSICR1_FB1_DMA_DONE_INTEN |
                  CSI_CSICR1_FB2_DMA_DONE_INTEN /* | CSI_CSICR1_SOF_INTEN */;

    // lets clear out anything that is cached.
    uint32_t creg, maskbits;

    creg = CSI_CSICR1;
    CSI_CSICR1 = (creg & ~CSI_CSICR1_FCC);                          // clear FCC bit
    maskbits = (CSI_CSICR1_CLR_STATFIFO) | (CSI_CSICR1_CLR_RXFIFO); // bits high for clearing stat and rx FIFOs
    CSI_CSICR1 = creg & (~CSI_CSICR1_FCC | maskbits);               // keep FCC off and clear FIFOs
    // wait while clear completes
    while ((CSI_CSICR1 & maskbits) != 0) {
    }
    CSI_CSICR1 = creg; // restore original status

    // wait until the clear is done.
    while (CSI_CSICR1 & (CSI_CSICR1_CLR_STATFIFO | CSI_CSICR1_CLR_RXFIFO)) {
    };

    // turn back on FCC
    CSI_CSICR1 = CSI_CSICR1;

    // And now lets start or restart the DMA
    CSI_CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF;

    // wait for this to complete.
    while (CSI_CSICR3 & CSI_CSICR3_DMA_REFLASH_RFF) {
    }

    // first clear, then set the RxFIFO full level to 16 double words or 64
    // bytes I think this level is chosen to match the FIFO size of the FlexSPI
    // used for the external PSRAM.
    CSI_CSICR3 = ((CSI_CSICR3 & ~CSI_CSICR3_RxFF_LEVEL(7)) |
                  CSI_CSICR3_RxFF_LEVEL(2)); // RxFF_LEVEL 2 is 16 double words

    // clear out any interrupt states.
    uint32_t csisr = CSI_CSISR;
    CSI_CSISR = csisr;

    // set this camera as the active one.
    active_dma_camera = this;
    attachInterruptVector(IRQ_CSI, &CSIInterrupt);
    NVIC_ENABLE_IRQ(IRQ_CSI);
    _cameraInput = CAMERA_INPUT_CSI;
    return true;
}

void print_csi_registers() {
    debug.printf("\tCSI_CSICR1: %x\n", CSI_CSICR1);
    debug.printf("\tCSI_CSICR2: %x\n", CSI_CSICR2);
    debug.printf("\tCSI_CSICR3: %x\n", CSI_CSICR3);
    debug.printf("\tCSI_CSISTATFIFO: %x\n", CSI_CSISTATFIFO);
    debug.printf("\tCSI_CSIRFIFO: %x\n", CSI_CSIRFIFO);
    debug.printf("\tCSI_CSIRXCNT: %x\n", CSI_CSIRXCNT);
    debug.printf("\tCSI_CSISR: %x\n", CSI_CSISR);
    debug.printf("\tCSI_CSIDMASA_STATFIFO: %x\n", CSI_CSIDMASA_STATFIFO);
    debug.printf("\tCSI_CSIDMATS_STATFIFO: %x\n", CSI_CSIDMATS_STATFIFO);
    debug.printf("\tCSI_CSIDMASA_FB1: %x\n", CSI_CSIDMASA_FB1);
    debug.printf("\tCSI_CSIDMASA_FB2: %x\n", CSI_CSIDMASA_FB2);
    debug.printf("\tCSI_CSIFBUF_PARA: %x\n", CSI_CSIFBUF_PARA);
    debug.printf("\tCSI_CSIIMAG_PARA: %x\n", CSI_CSIIMAG_PARA);
    debug.printf("\tCSI_CSICR18: %x\n", CSI_CSICR18);
    debug.printf("\tCSI_CSICR19: %x\n", CSI_CSICR19);
    debug.printf("\tCSI_CSIRFIFO: %x\n", CSI_CSIRFIFO);
};

volatile uint32_t CSIRxFIFOFullISRCount = 0;

size_t ImageSensor::readFrameCSI(void *buffer, size_t cb1, void *buffer2, size_t cb2) {

    if (_debug)
        debug.printf("$$ImageSensor::readFrameCSI(%p, %u, %p, %u)\n", buffer, cb1, buffer2, cb2);

    // If we are not using DMA, use helper function
    if (!_fuse_dma) {
        return readFrameCSI_use_FlexIO(buffer, cb1, buffer2, cb2);
    }

    changeCSIReadToFlexIOMode(false);

    DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);
    // Setup our buffer pointers.  Not sure if there is some place to store the
    // sizes?
    uint32_t frame_size_bytes = _width * _height * _bytesPerPixel;
    uint32_t return_value = frame_size_bytes;
    uint32_t csisr;

    CSI_CSIDMASA_FB1 = (uint32_t)buffer;
    // If only one buffer passed in use it for both CSI buffers...
    CSI_CSIDMASA_FB2 = buffer2 ? (uint32_t)buffer2 : (uint32_t)buffer;

    if (_dma_state == DMA_STATE_FRAME_ERROR) {
        if (_debug)
            debug.println("$$Reset DMA due to previous error");
        csi_reset_dma();
    }

    // Set Image Parameters--width in bytes and height
    CSI_CSIIMAG_PARA = CSI_CSIIMAG_PARA_IMAGE_WIDTH(_bytesPerPixel * _width) |
                       CSI_CSIIMAG_PARA_IMAGE_HEIGHT(_height);

    // Set our logical state
    _dma_state = DMA_STATE_ONE_FRAME;

    elapsedMillis timeout = 0; // reset the timeout
    //----------------------------------------------------------------------
    // DMA version
    //----------------------------------------------------------------------

    // Lets tell CSI to start the capture.
    // cear out any status bits set...
    csisr = CSI_CSISR;
    CSI_CSISR = csisr;

    // setup interrupts on DMA done on either buffer and maybe Start of frame.
    CSI_CSICR1 |= CSI_CSICR1_FB1_DMA_DONE_INTEN |
                  CSI_CSICR1_FB2_DMA_DONE_INTEN /* | CSI_CSICR1_SOF_INTEN */;
    CSI_CSICR1 |= CSI_CSICR1_RXFF_INTEN;     // lets add in interruptting on RXFIFO Full.
    CSI_CSICR3 |= CSI_CSICR3_DMA_REQ_EN_RFF; //  Disable RxFIFO DMA request
    CSIRxFIFOFullISRCount = 0;               // clear out the count...

    CSI_CSICR18 |= (CSI_CSICR18_CSI_ENABLE | CSI_CSICR18_BASEADDR_SWITCH_EN // CSI switches base addresses at frame start
                    | CSI_CSICR18_BASEADDR_CHANGE_ERROR_IE);                //  Enables base address error interrupt
#ifdef DEBUG_CAMERA
    if (_debug) {
        debug.print("After start capture\n");
        // Serial.printf("XCLK frequency: %u\n", _xclk_freq);
        print_csi_registers();
    }
#endif

    timeout = 0; // reset the timeout
    while (_dma_state == DMA_STATE_ONE_FRAME) {
        if (timeout > _timeout) {
            if (_debug)
                debug.println("Timeout waiting for CSI Frame complete");
#ifdef DEBUG_CAMERA
            print_csi_registers();
#endif
            return_value = 0; // error state.
            break;
        }
    }
    if (_debug)
        debug.printf("$$Count of RxFifoFull ISRs:%u\n", CSIRxFIFOFullISRCount);
    if (_dma_state == DMA_STATE_FRAME_ERROR) {

        // hackarama - lets see if I get a couple more interrupts before we return, will that help...
        uint16_t isr_count_entering = CSIRxFIFOFullISRCount;
        elapsedMicros emKludge;
        while ((emKludge < 1500) && ((CSIRxFIFOFullISRCount - isr_count_entering) < 3)) {
        }
        return_value = 0; // error state.
        if (_debug) {
            Serial.println("$$ImageSensor::readFrameCSI exited with Frame Error");
            print_csi_registers();
        }
    }
    // turn it off again?
    CSI_CSICR18 &= ~CSI_CSICR18_CSI_ENABLE;
    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);

    // maybe flush out frame buffer
    if ((uint32_t)_dma_last_completed_frame >= 0x20200000u)
        arm_dcache_flush_delete(_dma_last_completed_frame, frame_size_bytes);

    return return_value;
}

// checks and if necessary changes the mode to either CSI or FlexIO
bool ImageSensor::changeCSIReadToFlexIOMode(bool flexio_mode) {
    if (_csi_in_flexio_mode == flexio_mode)
        return true;

    //=========================================================================
    // Switch to FlexIO mode
    //=========================================================================
    if (flexio_mode) {
        // See if this is our first switch.
        // Switch to FlexIO mode.
        if (_pflex == nullptr) {
            if (_debug)
                debug.println("\tFirst Convert to FlexIO mode");
            // lets ontly try FlexIO3
            _pflex = FlexIOHandler::flexIOHandler_list[2];

            // lets map some of the pins over.
            _pflexio = &(_pflex->port());
            uint8_t tpclk_pin = _pflex->mapIOPinToFlexPin(_pclkPin);
            uint8_t thsync_pin = _pflex->mapIOPinToFlexPin(_hrefPin);
            uint8_t tg[8];
            for (uint8_t i = 0; i < 8; i++)
                tg[i] = _pflex->mapIOPinToFlexPin(_dPins[i]);

            if (_pflex->claimShifter(3))
                _fshifter = 3;
            else if (_pflex->claimShifter(7))
                _fshifter = 7;
            else {
                if (_debug)
                    debug.printf("OV767X Flex IO: Could not claim Shifter 3 or 7\n");
                _pflexio = nullptr;
                return false;
            }
            _fshifter_mask = 1 << _fshifter;
            // 3 does not have DMA
            //_dma_source = _pflex->shiftersDMAChannel(_fshifter); // looks like they use
            // Now request one timer
            // BUGBUG< check other usage
            _ftimer = _pflex->requestTimers(); // request 1 timer.
            if (_ftimer == 0xff) {
                if (_debug)
                    debug.printf("ImageSensor Flex IO: failed to request timer\n");
                _pflexio = nullptr;
                return false;
            }

            if (_debug) {
                debug.printf("\tPCLK:%x hsync:%x data pin: ", tpclk_pin, thsync_pin);
                for (uint8_t i = 0; i < 8; i++)
                    debug.printf(" %x", tg[i]);
                debug.println();
            }

            // We are always 8 bit mode
            // For more notes see the flexio_configure function
            _pflexio->SHIFTCFG[_fshifter] = FLEXIO_SHIFTCFG_PWIDTH(7);
            _pflexio->TIMCMP[_ftimer] = (8U * 1) - 1;

            _pflexio->TIMCTL[_ftimer] =
                FLEXIO_TIMCTL_TIMOD(3) |
                FLEXIO_TIMCTL_PINSEL(tpclk_pin) // "Pin" is 16 = PCLK
                //| FLEXIO_TIMCTL_TRGSEL(4 * (thsync_pin/2)) // "Trigger" is 12 =
                // HSYNC
                | FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(
                      thsync_pin)) // "Trigger" is 12 = HSYNC
                | FLEXIO_TIMCTL_TRGSRC;

            // BUGBUG:: the pins are in reversed order... Arg...
            _pflexio->SHIFTCTL[_fshifter] = FLEXIO_SHIFTCTL_TIMSEL(_ftimer) |
                                            FLEXIO_SHIFTCTL_SMOD(1) |
                                            FLEXIO_SHIFTCTL_PINSEL(tg[7]);
            _pflexio->TIMCFG[_ftimer] =
                FLEXIO_TIMCFG_TIMOUT(1) | FLEXIO_TIMCFG_TIMDEC(2) |
                FLEXIO_TIMCFG_TIMENA(6) | FLEXIO_TIMCFG_TIMDIS(6);

            // CTRL, page 2916
            _pflexio->CTRL = FLEXIO_CTRL_FLEXEN; // enable after everything configured
#ifdef DEBUG_FLEXIO
            debug.printf(" FLEXIO:%u Shifter:%u Timer:%u\n", _pflex->FlexIOIndex(),
                         _fshifter, _ftimer);
            debug.print("     SHIFTCFG = ");
            debug.printf(" %08X", _pflexio->SHIFTCFG[_fshifter]);
            debug.print("\n     SHIFTCTL = ");
            debug.printf(" %08X", _pflexio->SHIFTCTL[_fshifter]);
            debug.printf("\n     TIMCMP = %08X\n", _pflexio->TIMCMP[_ftimer]);
            debug.printf("     TIMCFG = %08X\n", _pflexio->TIMCFG[_ftimer]);
            debug.printf("     TIMCTL = %08X\n", _pflexio->TIMCTL[_ftimer]);
#endif
        }
        // We then need to make sure all of the IO pins are in the
        // reight state.
        _pflex->setIOPinToFlexMode(_hrefPin);
        _pflex->setIOPinToFlexMode(_pclkPin);
        for (uint8_t i = 0; i < 8; i++)
            _pflex->setIOPinToFlexMode(_dPins[i]);
        _pflex->setClockSettings(3, 1, 1);

        // Set vsync pin to intput
        pinMode(_vsyncPin, INPUT /*INPUT_PULLDOWN*/);
    }
    //=========================================================================
    // Switch back to CSI mode
    //=========================================================================
    else {
        // start off trying to restore the right pin modes and
        // see if that is enought
        configureCSIPin(_vsyncPin);
        configureCSIPin(_hrefPin);
        configureCSIPin(_pclkPin);

        for (uint8_t i = 0; i < 8; i++)
            configureCSIPin(_dPins[i]);
    }

    _csi_in_flexio_mode = flexio_mode;
    return true;
}

size_t ImageSensor::readFrameCSI_use_FlexIO(void *buffer, size_t cb1, void *buffer2, size_t cb2) {

    if (_debug)
        debug.printf("$$ImageSensor::readFrameCSI_use_FlexIO(%p, %u, %p, %u)\n", buffer, cb1, buffer2, cb2);

    changeCSIReadToFlexIOMode(true);
    // This is a duplicate of most of some of the readFrameFlexIO
    // maybe later split off the none DMA version to subfunction.

    DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);

    uint32_t frame_size_bytes = _width * _height * _bytesPerPixel;
    if (_format == pixformat_t::JPEG) {
        frame_size_bytes = frame_size_bytes / 5;
    } else if ((cb1 + cb2) < frame_size_bytes) {
        DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);
        return 0; // not enough to hold normal images...
    }

    // flexio_configure(); // one-time hardware setup
    // wait for VSYNC to go high and then low with a sort of glitch filter
    elapsedMillis emWaitSOF;
    elapsedMicros emGlitch;
    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);

    for (;;) {
        if (emWaitSOF > _timeout) {
            if (_debug)
                debug.println("Timeout waiting for Start of Frame");
            return 0;
        }
        while ((*_vsyncPort & _vsyncMask) == 0) {
            if (emWaitSOF > _timeout) {
                if (_debug)
                    debug.println(
                        "Timeout waiting for rising edge Start of Frame");
                return 0;
            }
        }
        emGlitch = 0;
        while ((*_vsyncPort & _vsyncMask) != 0) {
            if (emWaitSOF > _timeout) {
                if (_debug)
                    debug.println(
                        "Timeout waiting for falling edge Start of Frame");
                return 0;
            }
        }
        if (emGlitch > 5)
            break;
    }
    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;
    uint32_t *p = (uint32_t *)buffer;
    DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);

    elapsedMillis timeout = 0;

    // for flexIO will simply return the first buffer.
    //_dma_last_completed_frame = buffer;

    //----------------------------------------------------------------------
    // Polling FlexIO version
    //----------------------------------------------------------------------
    if (_debug)
        debug.println("\tNot DMA");
    // lets try another version of this to see if cleaner.
    uint32_t *p_end = (uint32_t *)((uint8_t *)p + cb1);
    uint32_t max_time_to_wait = _timeout;

    uint32_t frame_bytes_received = 0;
    while (frame_bytes_received < frame_size_bytes) {
        while ((_pflexio->SHIFTSTAT & _fshifter_mask) == 0) {
            if (timeout > max_time_to_wait) {
                DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);

                if (_debug)
                    debug.printf(
                        "Timeout between characters: received: %u bytes\n",
                        frame_bytes_received);
                // wait for FlexIO shifter data
                DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);
                break;
            }
        }
        // Lets simplify back to single shifter for now
        uint8_t *pu8 = (uint8_t *)p;
        // try the bitbyte swapped version.
        *p++ = _pflexio->SHIFTBUFBBS[_fshifter];
        if ((_format == pixformat_t::JPEG) && (frame_bytes_received > 0)) {
            // jpeg check for
            for (int i = 0; i < 4; i++) {
                if ((pu8[i - 1] == 0xff) && (pu8[i] == 0xd9)) {
                    if (_debug)
                        Serial.printf("JPEG - found end marker at %u\n",
                                      frame_bytes_received + i);
                    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);
                    return frame_bytes_received + i + 1;
                }
            }
        }

        frame_bytes_received += sizeof(uint32_t);
        // Check to see if we filled current buffer
        if (p >= p_end) {
            // yes, now see if we have another
            if (buffer2) {
                p = (uint32_t *)buffer2;
                p_end = (uint32_t *)((uint8_t *)p + cb2);
                buffer2 = nullptr;
            } else {
                if (_debug && (frame_bytes_received < frame_size_bytes))
                    Serial.printf(
                        "Error: Image size exceeded buffer space\n");
                break;
            }
        }
        max_time_to_wait = 50; // maybe second setting.
        timeout = 0;           // reset timeout.
    }

    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);
    return frame_bytes_received;
}


bool ImageSensor::startReadCSI(bool (*callback)(void *frame_buffer), void *fb1,
                               size_t cb1, void *fb2, size_t cb2) {
    if (_debug)
        debug.printf("$$ImageSensor::startReadCSI(%p: %p, %u, %p, %u)\n", callback, fb1, cb1, fb2, cb2);

    changeCSIReadToFlexIOMode(false);

    DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);
    // Setup our buffer pointers.  Not sure if there is some place to store the
    // sizes?

    _frame_buffer_1 = (uint8_t *)fb1;
    _frame_buffer_1_size = cb1;
    _frame_buffer_2 = (uint8_t *)fb2;
    _frame_buffer_2_size = cb2;
    _callback = callback;
    active_dma_camera = this;

    CSI_CSIDMASA_FB1 = (uint32_t)fb1;
    CSI_CSIDMASA_FB2 = fb2 ? (uint32_t)fb2 : (uint32_t)fb1;
    // Set Image Parameters--width in bytes and height
    CSI_CSIIMAG_PARA = CSI_CSIIMAG_PARA_IMAGE_WIDTH(_bytesPerPixel * _width) |
                       CSI_CSIIMAG_PARA_IMAGE_HEIGHT(_height);

    // Set our logical state
    _dma_state = DMASTATE_RUNNING;

    // Lets tell CSI to start the capture.
    CSI_CSICR18 |= (CSI_CSICR18_CSI_ENABLE | CSI_CSICR18_BASEADDR_SWITCH_EN // CSI switches base addresses at frame start
                    | CSI_CSICR18_BASEADDR_CHANGE_ERROR_IE);                //  Enables base address error interrupt
#ifdef DEBUG_CAMERA
    debug.print("After start capture\n");
    // Serial.printf("XCLK frequency: %u\n", _xclk_freq);
    print_csi_registers();
#endif

    return true;
}

// not implemented yet
bool ImageSensor::stopReadCSI() {
    // turn it off again?
    CSI_CSICR18 &= ~CSI_CSICR18_CSI_ENABLE;
    _dma_state = DMA_STATE_STOPPED;
    return true;
}

void ImageSensor::CSIInterrupt() {
    // debug.printf("static CSII:%x\n", CSI_CSISR);
    DBGdigitalWriteFast(3, HIGH);
    active_dma_camera->processCSIInterrupt();
    DBGdigitalWriteFast(3, LOW);
}

void ImageSensor::processCSIInterrupt() {
    // lets grab the state and see what the ISR was for....
    uint32_t csisr = CSI_CSISR; //
    //if (_debug /*&& (csisr & (CSI_CSISR_BASEADDR_CHHANGE_ERROR | CSI_CSISR_SOF_INT | CSI_CSISR_DMA_TSF_DONE_FB1 | CSI_CSISR_DMA_TSF_DONE_FB2))*/)
    //    debug.printf("CSII:%x\n", csisr);
    uint32_t frame_size_bytes = _width * _height * _bytesPerPixel;

    // Not sure if we will not SOF or not but...
    if (csisr & CSI_CSISR_SOF_INT) {
    }

    // experiment keep track of how many times this was called.
    // if (csisr & CSI_CSISR_RxFF_INT) {
    CSIRxFIFOFullISRCount++; //
    //}

    if (_dma_state == DMA_STATE_ONE_FRAME) {
        if (csisr & CSI_CSISR_BASEADDR_CHHANGE_ERROR) {
            // We ran into an error
            _dma_state = DMA_STATE_FRAME_ERROR;

        } else if (csisr & CSI_CSISR_DMA_TSF_DONE_FB1) {
            _dma_last_completed_frame = (uint8_t *)CSI_CSIDMASA_FB1;
            _dma_last_completed_image_size = frame_size_bytes;
            _dma_state = DMA_STATE_STOPPED;
        } else if (csisr & CSI_CSISR_DMA_TSF_DONE_FB2) {
            _dma_last_completed_frame = (uint8_t *)CSI_CSIDMASA_FB2;
            _dma_last_completed_image_size = frame_size_bytes;
            _dma_state = DMA_STATE_STOPPED;
        }

    } else if (_dma_state == DMASTATE_RUNNING) {
        if (csisr & CSI_CSISR_DMA_TSF_DONE_FB1) {
            _dma_last_completed_frame = (uint8_t *)CSI_CSIDMASA_FB1;
            _dma_last_completed_image_size = frame_size_bytes;

            if ((uint32_t)_dma_last_completed_frame >= 0x20200000u)
                arm_dcache_flush_delete(_dma_last_completed_frame, frame_size_bytes);
            if (_callback)
                (*_callback)(_dma_last_completed_frame); // TODO: use EventResponder
        } else if (csisr & CSI_CSISR_DMA_TSF_DONE_FB2) {
            _dma_last_completed_frame = (uint8_t *)CSI_CSIDMASA_FB2;
            if ((uint32_t)_dma_last_completed_frame >= 0x20200000u)
                arm_dcache_flush_delete(_dma_last_completed_frame, frame_size_bytes);
            if (_callback)
                (*_callback)(_dma_last_completed_frame); // TODO: use EventResponder
        }
    }
    CSI_CSISR = csisr;
    asm("DSB"); // hopefully keeps from being called again
}

//=============================================================================
// Reading using GPIO support
//=============================================================================
size_t ImageSensor::readFrameGPIO(void *buffer, size_t cb1, void *buffer2,
                                  size_t cb2) {
    debug.printf("$$ImageSensor::readFrameGPIO(%p, %u, %p, %u)\n", buffer, cb1,
                 buffer2, cb2);
    const uint32_t frame_size_bytes = _width * _height * _bytesPerPixel;

    if ((cb1 + cb2) < frame_size_bytes)
        return 0; // not enough to hold image
    DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);

    uint8_t *b = (uint8_t *)buffer;
    uint32_t cb = (uint32_t)cb1;
    //  bool _grayscale;  // ????  member variable ?????????????
    int bytesPerRow = _width * _bytesPerPixel;

    // Falling edge indicates start of frame
    // pinMode(PCLK_PIN, INPUT); // make sure back to input pin...
    // lets add our own glitch filter.  Say it must be hig for at least
    // 100us

    delayMicroseconds(5); // debug for digitalWrite
    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);
    elapsedMicros emHigh;
    do {
        while ((*_vsyncPort & _vsyncMask) == 0)
            ; // wait for HIGH
        emHigh = 0;
        while ((*_vsyncPort & _vsyncMask) != 0)
            ; // wait for LOW
    } while (emHigh < 2);
    DBGdigitalWriteFast(DBG_TIMING_PIN, HIGH);

    for (int i = 0; i < _height; i++) {
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

            // uint32_t in = ((_frame_buffer_pointer)? GPIO1_DR :
            // GPIO6_DR) >> 18; // read all bits in parallel
            uint32_t in = (GPIO7_PSR >> 4); // read all bits in parallel

            // uint32_t in = mmBus;
            // bugbug what happens to the the data if grayscale?
            if (!(j & 1) || !_grayscale) {
                *b++ = in;

                if (buffer2 && (--cb == 0)) {
                    if (_debug)
                        debug.printf("\t$$ 2nd buffer: %u %u\n", i, j);
                    b = (uint8_t *)buffer2;
                    cb = (uint32_t)cb2;
                    buffer2 = nullptr;
                }
            }
            while (((*_pclkPort & _pclkMask) != 0) &&
                   ((*_hrefPort & _hrefMask) != 0))
                ; // wait for LOW bail if _href is lost
        }

        while ((*_hrefPort & _hrefMask) != 0)
            ; // wait for LOW
        interrupts();
    }
    DBGdigitalWriteFast(DBG_TIMING_PIN, LOW);
    return frame_size_bytes;
}
