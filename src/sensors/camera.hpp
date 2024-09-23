#ifndef CAMERA_H
#define CAMERA_H

#include <imxrt.h>

void enable_csi() {
    // setup sCSI[9:2] select daisy registers
    // set last bit to zero
    IOMUXC_CSI_DATA02_SELECT_INPUT &= ~0x1U; // AD_B1_15
    IOMUXC_CSI_DATA03_SELECT_INPUT &= ~0x1U; // AD_B1_14
    IOMUXC_CSI_DATA04_SELECT_INPUT &= ~0x1U; // AD_B1_13
    IOMUXC_CSI_DATA05_SELECT_INPUT &= ~0x1U; // AD_B1_12   
    IOMUXC_CSI_DATA06_SELECT_INPUT &= ~0x1U; // AD_B1_11
    IOMUXC_CSI_DATA07_SELECT_INPUT &= ~0x1U; // AD_B1_10
    IOMUXC_CSI_DATA08_SELECT_INPUT &= ~0x1U; // AD_B1_09
    IOMUXC_CSI_DATA09_SELECT_INPUT &= ~0x1U; // AD_B1_08

    // Set SW_MUX_CTL_PAD_X all to ALT4 (ALT4 corresponds to CSI)
    // set the last 5 bits to 10100
    // SION enable and mux mode alt 4
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = (IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 & ~0x1FU) | 0x14U;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = (IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 & ~0x1FU) | 0x14U;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_13 = (IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_13 & ~0x1FU) | 0x14U;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_12 = (IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_12 & ~0x1FU) | 0x14U;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11 = (IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11 & ~0x1FU) | 0x14U;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10 = (IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10 & ~0x1FU) | 0x14U;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = (IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 & ~0x1FU) | 0x14U;
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = (IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 & ~0x1FU) | 0x14U;

    // set SW_PAD_CTL_PAD_X defaults
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_15 = (IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_15 & ~0x1F8F9U) | 0x010B0U;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_14 = (IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_14 & ~0x1F8F9U) | 0x010B0U;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_13 = (IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_13 & ~0x1F8F9U) | 0x010B0U;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_12 = (IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_12 & ~0x1F8F9U) | 0x010B0U;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_11 = (IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_11 & ~0x1F8F9U) | 0x010B0U;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_10 = (IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_10 & ~0x1F8F9U) | 0x010B0U;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09 = (IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09 & ~0x1F8F9U) | 0x010B0U;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_08 = (IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_08 & ~0x1F8F9U) | 0x010B0U;

    // enable csi_clk_enable
    // set bits [3:2] to 1
    CCM_CCGR2 |= 0x6U;

    // hardware reset

    // config all regs

    // enable csi
}

#endif