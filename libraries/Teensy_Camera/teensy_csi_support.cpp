#include "teensy_csi_support.h"

#define PAD_INPUT (IOMUXC_PAD_DSE(7))
#define PAD_INPUT_PU (IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS)
#define PAD_INPUT_PD (IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(0) | IOMUXC_PAD_HYS)

const PROGMEM csi_pain_mapping_t csi_pin_mapping_table[] = {
    // VSYNC
    {CSI_VS, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06, 0x4U,
     &IOMUXC_CSI_VSYNC_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_06, PAD_INPUT_PD},
    {CSI_VS, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13, 0x2U,
     &IOMUXC_CSI_VSYNC_SELECT_INPUT, 0x2U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_13, PAD_INPUT_PD},
    {CSI_VS, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_14, 0x4U,
     &IOMUXC_CSI_VSYNC_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_14, PAD_INPUT_PD},
    // HSYNC
    {CSI_HS, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07, 0x4U,
     &IOMUXC_CSI_HSYNC_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07, PAD_INPUT},
    {CSI_HS, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_14, 0x2U,
     &IOMUXC_CSI_HSYNC_SELECT_INPUT, 0x2U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_14, PAD_INPUT},
    {CSI_HS, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_15, 0x4U,
     &IOMUXC_CSI_HSYNC_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_15, PAD_INPUT},
    // DCLK or Pixel Clk
    {CSI_PCLK, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04, 0x4U,
     &IOMUXC_CSI_PIXCLK_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04, PAD_INPUT_PD},
    {CSI_PCLK, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12, 0x2U,
     &IOMUXC_CSI_PIXCLK_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_12, PAD_INPUT_PD},
    // MCLK
    {CSI_MCLK, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05, 0x4U, 0, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_05, IOMUXC_PAD_DSE(7)},
    {CSI_MCLK, &IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_15, 0x2U, 0, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_15, IOMUXC_PAD_DSE(7)},

    // D7-D0
    {CSI_D02, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15, 0x4U,
     &IOMUXC_CSI_DATA02_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_15, PAD_INPUT},
    {CSI_D02, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_11, 0x4U,
     &IOMUXC_CSI_DATA02_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_11, PAD_INPUT},

    {CSI_D03, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14, 0x4U,
     &IOMUXC_CSI_DATA03_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_14, PAD_INPUT},
    {CSI_D03, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_10, 0x4U,
     &IOMUXC_CSI_DATA03_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_10, PAD_INPUT},

    {CSI_D04, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_13, 0x4U,
     &IOMUXC_CSI_DATA04_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_13, PAD_INPUT},
    {CSI_D04, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_09, 0x4U,
     &IOMUXC_CSI_DATA04_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_09, PAD_INPUT},

    {CSI_D05, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_12, 0x4U,
     &IOMUXC_CSI_DATA05_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_12, PAD_INPUT},
    {CSI_D05, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_08, 0x4U,
     &IOMUXC_CSI_DATA05_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_08, PAD_INPUT},

    {CSI_D06, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11, 0x4U,
     &IOMUXC_CSI_DATA06_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_11, PAD_INPUT},
    {CSI_D06, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_07, 0x4U,
     &IOMUXC_CSI_DATA06_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07, PAD_INPUT},

    {CSI_D07, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10, 0x4U,
     &IOMUXC_CSI_DATA07_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_10, PAD_INPUT},
    {CSI_D07, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_06, 0x4U,
     &IOMUXC_CSI_DATA07_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_06, PAD_INPUT},

    {CSI_D08, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09, 0x4U,
     &IOMUXC_CSI_DATA08_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09, PAD_INPUT},
    {CSI_D08, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_05, 0x4U,
     &IOMUXC_CSI_DATA08_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_05, PAD_INPUT},

    {CSI_D09, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08, 0x4U,
     &IOMUXC_CSI_DATA09_SELECT_INPUT, 0x0U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_08, PAD_INPUT},
    {CSI_D09, &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_04, 0x4U,
     &IOMUXC_CSI_DATA09_SELECT_INPUT, 0x1U,
     &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_04, PAD_INPUT},
};

const csi_pain_mapping_t *mapPinToCSIPinInfo(uint8_t pin) {
    if (pin >= CORE_NUM_DIGITAL)
        return nullptr; // pin is out of range.
    volatile uint32_t *pin_mux = portConfigRegister(pin);
    Serial.printf("MAP Pin %u(%p)\n", pin, pin_mux);
    for (uint8_t i = 0;
         i < (sizeof(csi_pin_mapping_table) / sizeof(csi_pin_mapping_table[0]));
         i++) {
        // Serial.printf("\t%xp %p\n", csi_pin_mapping_table[i].mux,
        //               csi_pin_mapping_table[i].pad);
        if (csi_pin_mapping_table[i].mux == pin_mux) {
            return &csi_pin_mapping_table[i];
        }
    }
    return nullptr;
}

bool verifyCSIPin(uint8_t pin, uint8_t csi_usage) {
    const csi_pain_mapping_t *pcpm = mapPinToCSIPinInfo(pin);

    bool return_value = ((pcpm != nullptr) && (pcpm->csi_usage == csi_usage));
    if (!return_value) {
        if (pcpm == nullptr)
            Serial.printf("Warning pin: %u is not a known CSI pin\n", pin);
        else
            Serial.printf("Warning pin:%u is a CSI pin, but does not have the "
                          "right usage: %u!=%u\n",
                          pin, csi_usage, pcpm->csi_usage);
    }
    return return_value;
}

extern bool configureCSIPin(uint8_t pin) {
    const csi_pain_mapping_t *pcpm = mapPinToCSIPinInfo(pin);
    // Serial.printf("Configure CSIPin %u\n", pin);
    if (pcpm == nullptr)
        return false;
    // Lets setup the mux
    *pcpm->mux = pcpm->mux_val;
    // Serial.printf("\t%p = %x\n", pcpm->mux, pcpm->mux_val);

    // If it has an input select set it also
    if (pcpm->pin_select_input) {
        *pcpm->pin_select_input = pcpm->select_input_val;
        // Serial.printf("\t%p = %x\n", pcpm->pin_select_input, pcpm->select_input_val);
    }

    // Lets setup the pad
    *pcpm->pad = pcpm->pad_val;
    // Serial.printf("\t%p = %x\n", pcpm->pad, pcpm->pad_val);
    return true;
}
