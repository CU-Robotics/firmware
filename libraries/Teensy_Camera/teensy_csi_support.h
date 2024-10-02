#pragma once

#include "Teensy_Camera.h"
#include <Arduino.h>

enum {
    CSI_D02 = 2,
    CSI_D03,
    CSI_D04,
    CSI_D05,
    CSI_D06,
    CSI_D07,
    CSI_D08,
    CSI_D09,
    CSI_VS = 100,
    CSI_HS,
    CSI_PCLK,
    CSI_MCLK
};

typedef struct csi_pin_mapping_setup { // not really bitband, old name from
                                       // Teensy3
    uint8_t csi_usage;
    volatile uint32_t *mux;
    uint32_t mux_val;
    volatile uint32_t *pin_select_input;
    uint32_t select_input_val;
    volatile uint32_t *pad;
    uint32_t pad_val;
} csi_pain_mapping_t;

extern const csi_pain_mapping_t *mapPinToCSIPinInfo(uint8_t pin);

extern bool verifyCSIPin(uint8_t pin, uint8_t csi_usage);
extern bool configureCSIPin(uint8_t pin);
