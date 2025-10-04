/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for
 * details.
 *
 * OV5640 register definitions.
 *
 * Note: Some mods made to get JPEG working and other minor things, like
 * special effects, white balance, night mode, hue, sharpness.
 */

#ifndef __REG_REGS_H__
#define __REG_REGS_H__

// Image sensor drivers configuration.
#define OMV_OV5640_AF_ENABLE (0)
#define OMV_OV5640_PLL_CTRL2 (0x64)
#define OMV_OV5640_PLL_CTRL3 (0x13)
#define OMV_OV5640_REV_Y_CHECK (0)
#define OMV_OV5640_REV_Y_FREQ (25000000)
#define OMV_OV5640_REV_Y_CTRL2 (0x54)
#define OMV_OV5640_REV_Y_CTRL3 (0x13)

#define SYSTEM_RESET_00 0x3000
#define SYSTEM_RESET_01 0x3001
#define SYSTEM_RESET_02 0x3002

#define CLOCK_ENABLE_02 0x3006

#define SYSTEM_CTROL0 0x3008

#define AF_CMD_MAIN 0x3022
#define AF_CMD_ACK 0x3023
#define AF_FW_STATUS 0x3029

#define SC_PLL_CONTRL0 0x3034
#define SC_PLL_CONTRL1 0x3035
#define SC_PLL_CONTRL2 0x3036
#define SC_PLL_CONTRL3 0x3037

#define SCCB_SYSTEM_CTRL_1 0x3103
#define SYSTEM_ROOT_DIVIDER 0x3108

#define AWB_R_GAIN_H 0x3400
#define AWB_R_GAIN_L 0x3401

#define AWB_G_GAIN_H 0x3402
#define AWB_G_GAIN_L 0x3403

#define AWB_B_GAIN_H 0x3404
#define AWB_B_GAIN_L 0x3405

#define AWB_MANUAL_CONTROL 0x3406

#define AEC_PK_EXPOSURE_0 0x3500
#define AEC_PK_EXPOSURE_1 0x3501
#define AEC_PK_EXPOSURE_2 0x3502

#define AEC_PK_MANUAL 0x3503

#define AEC_PK_REAL_GAIN_H 0x350A
#define AEC_PK_REAL_GAIN_L 0x350B

#define TIMING_HS_H 0x3800
#define TIMING_HS_L 0x3801

#define TIMING_VS_H 0x3802
#define TIMING_VS_L 0x3803

#define TIMING_HW_H 0x3804
#define TIMING_HW_L 0x3805

#define TIMING_VH_H 0x3806
#define TIMING_VH_L 0x3807

#define TIMING_DVPHO_H 0x3808
#define TIMING_DVPHO_L 0x3809

#define TIMING_DVPVO_H 0x380A
#define TIMING_DVPVO_L 0x380B

#define TIMING_HTS_H 0x380C
#define TIMING_HTS_L 0x380D

#define TIMING_VTS_H 0x380E
#define TIMING_VTS_L 0x380F

#define TIMING_HOFFSET_H 0x3810
#define TIMING_HOFFSET_L 0x3811

#define TIMING_VOFFSET_H 0x3812
#define TIMING_VOFFSET_L 0x3813

#define TIMING_X_INC 0x3814
#define TIMING_Y_INC 0x3815

#define TIMING_TC_REG_20 0x3820
#define TIMING_TC_REG_21 0x3821

#define AEC_CTRL_00 0x3A00

#define AEC_GAIN_CEILING_H 0x3A18
#define AEC_GAIN_CEILING_L 0x3A18

#define BLC_CTRL_00 0x4000

#define BLACK_LEVEL_00_H 0x402C
#define BLACK_LEVEL_00_L 0x402D

#define BLACK_LEVEL_01_H 0x402E
#define BLACK_LEVEL_01_L 0x402F

#define BLACK_LEVEL_10_H 0x4030
#define BLACK_LEVEL_10_L 0x4031

#define BLACK_LEVEL_11_H 0x4032
#define BLACK_LEVEL_11_L 0x4033

#define FORMAT_CONTROL 0x4300

#define VFIFO_HSIZE_H 0x4602
#define VFIFO_HSIZE_L 0x4603

#define VFIFO_VSIZE_H 0x4604
#define VFIFO_VSIZE_L 0x4605
#define JPEG_MODE_SEL 0x4713

#define JPEG_CTRL07 0x4407
#define COMPRESSION_ENABLE_b5 0x3821

#define ISP_CONTROL_00 0x5300

#define FORMAT_CONTROL_MUX 0x501F

#define PRE_ISP_TEST 0x503D

#define ISP_CONTROL_01 0x5001
#define SDE_CTRL0 0x5580
#define SDE_CTRL1 0x5581
#define SDE_CTRL2 0x5582
#define SDE_CTRL5 0x5585
#define SDE_CTRL6 0x5586
#define SDE_CTRL8 0x5588

#define CIP_SHARPENMT_THRESH1 0x5300
#define CIP_SHARPENMT_THRESH2 0x5301
#define CIP_SHARPENMT_OFFSET1 0x5302
#define CIP_SHARPENMT_OFFSET2 0x5303
#define CIP_CTRL 0x5308
#define CIP_SHARPENTH_THRESH1 0x5309
#define CIP_SHARPENTH_THRESH2 0x530A
#define CIP_SHARPENTH_OFFSET1 0x530B
#define CIP_SHARPENTH_OFFSET2 0x530C

#define MCU_FIRMWARE_BASE 0x8000

/* Autofocus - see Omnivision Embedded Firmware User Guide - VCM AF Module */
#define OV5640_CMD_MAIN 0x3022
#define OV5640_CMD_ACK 0x3023
#define OV5640_CMD_PARA0 0x3024
#define OV5640_CMD_PARA1 0x3025
#define OV5640_CMD_PARA2 0x3026
#define OV5640_CMD_PARA3 0x3027
#define OV5640_CMD_PARA4 0x3028
#define OV5640_CMD_FW_STATUS 0x3029

#define AF_TRIG_SINGLE_AUTO_FOCUS 0x03
#define AF_CONTINUE_AUTO_FOCUS 0x04
#define AF_SET_VCM_STEP 0x1A
#define AF_GET_VCM_STEP 0x1B

#define AF_STATUS_S_FIRMWARE 0x7F
#define AF_STATUS_S_STARTUP 0x7E
#define AF_STATUS_S_IDLE 0x70
#define AF_STATUS_S_FOCUSED 0x10
#define AF_STATUS_S_ZONE_CONFIG 0x16
#define AF_STATUS_S_FOCUSING 0x00 // 0x00 - 0X0F, 0x80 - 0X8F

#endif //__REG_REGS_H__