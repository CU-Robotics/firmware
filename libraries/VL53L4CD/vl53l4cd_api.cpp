/**
 ******************************************************************************
 * @file    vl53l4cd_api.cpp
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    29 November 2021
 * @brief   Implementation of the VL53L4CD APIs.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "vl53l4cd_class.h"
#include <string.h>
#include <math.h>

static const uint8_t VL53L4CD_DEFAULT_CONFIGURATION[] = {
  0x12, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C),
   else don't touch */
  0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1
   (pull up at AVDD) */
  0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1
  (pull up at AVDD) */
  0x11, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low
  (bits 3:0 must be 0x1), use SetInterruptPolarity() */
  0x02, /* 0x31 : bit 1 = interrupt depending on the polarity,
  use CheckForDataReady() */
  0x00, /* 0x32 : not user-modifiable */
  0x02, /* 0x33 : not user-modifiable */
  0x08, /* 0x34 : not user-modifiable */
  0x00, /* 0x35 : not user-modifiable */
  0x08, /* 0x36 : not user-modifiable */
  0x10, /* 0x37 : not user-modifiable */
  0x01, /* 0x38 : not user-modifiable */
  0x01, /* 0x39 : not user-modifiable */
  0x00, /* 0x3a : not user-modifiable */
  0x00, /* 0x3b : not user-modifiable */
  0x00, /* 0x3c : not user-modifiable */
  0x00, /* 0x3d : not user-modifiable */
  0xff, /* 0x3e : not user-modifiable */
  0x00, /* 0x3f : not user-modifiable */
  0x0F, /* 0x40 : not user-modifiable */
  0x00, /* 0x41 : not user-modifiable */
  0x00, /* 0x42 : not user-modifiable */
  0x00, /* 0x43 : not user-modifiable */
  0x00, /* 0x44 : not user-modifiable */
  0x00, /* 0x45 : not user-modifiable */
  0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high,
  2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
  0x0b, /* 0x47 : not user-modifiable */
  0x00, /* 0x48 : not user-modifiable */
  0x00, /* 0x49 : not user-modifiable */
  0x02, /* 0x4a : not user-modifiable */
  0x14, /* 0x4b : not user-modifiable */
  0x21, /* 0x4c : not user-modifiable */
  0x00, /* 0x4d : not user-modifiable */
  0x00, /* 0x4e : not user-modifiable */
  0x05, /* 0x4f : not user-modifiable */
  0x00, /* 0x50 : not user-modifiable */
  0x00, /* 0x51 : not user-modifiable */
  0x00, /* 0x52 : not user-modifiable */
  0x00, /* 0x53 : not user-modifiable */
  0xc8, /* 0x54 : not user-modifiable */
  0x00, /* 0x55 : not user-modifiable */
  0x00, /* 0x56 : not user-modifiable */
  0x38, /* 0x57 : not user-modifiable */
  0xff, /* 0x58 : not user-modifiable */
  0x01, /* 0x59 : not user-modifiable */
  0x00, /* 0x5a : not user-modifiable */
  0x08, /* 0x5b : not user-modifiable */
  0x00, /* 0x5c : not user-modifiable */
  0x00, /* 0x5d : not user-modifiable */
  0x01, /* 0x5e : not user-modifiable */
  0xcc, /* 0x5f : not user-modifiable */
  0x07, /* 0x60 : not user-modifiable */
  0x01, /* 0x61 : not user-modifiable */
  0xf1, /* 0x62 : not user-modifiable */
  0x05, /* 0x63 : not user-modifiable */
  0x00, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB),
   use SetSigmaThreshold(), default value 90 mm  */
  0xa0, /* 0x65 : Sigma threshold LSB */
  0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB),
   use SetSignalThreshold() */
  0x80, /* 0x67 : Min count Rate LSB */
  0x08, /* 0x68 : not user-modifiable */
  0x38, /* 0x69 : not user-modifiable */
  0x00, /* 0x6a : not user-modifiable */
  0x00, /* 0x6b : not user-modifiable */
  0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register,
   use SetIntermeasurementInMs() */
  0x00, /* 0x6d : Intermeasurement period */
  0x0f, /* 0x6e : Intermeasurement period */
  0x89, /* 0x6f : Intermeasurement period LSB */
  0x00, /* 0x70 : not user-modifiable */
  0x00, /* 0x71 : not user-modifiable */
  0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB),
   use SetD:tanceThreshold() */
  0x00, /* 0x73 : distance threshold high LSB */
  0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB),
   use SetD:tanceThreshold() */
  0x00, /* 0x75 : distance threshold low LSB */
  0x00, /* 0x76 : not user-modifiable */
  0x01, /* 0x77 : not user-modifiable */
  0x07, /* 0x78 : not user-modifiable */
  0x05, /* 0x79 : not user-modifiable */
  0x06, /* 0x7a : not user-modifiable */
  0x06, /* 0x7b : not user-modifiable */
  0x00, /* 0x7c : not user-modifiable */
  0x00, /* 0x7d : not user-modifiable */
  0x02, /* 0x7e : not user-modifiable */
  0xc7, /* 0x7f : not user-modifiable */
  0xff, /* 0x80 : not user-modifiable */
  0x9B, /* 0x81 : not user-modifiable */
  0x00, /* 0x82 : not user-modifiable */
  0x00, /* 0x83 : not user-modifiable */
  0x00, /* 0x84 : not user-modifiable */
  0x01, /* 0x85 : not user-modifiable */
  0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
  0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(),
   If you want an automatic start after VL53L4CD_init() call,
    put 0x40 in location 0x87 */
};

VL53L4CD_ERROR VL53L4CD::VL53L4CD_GetSWVersion(
  VL53L4CD_Version_t *p_Version)
{
  VL53L4CD_ERROR Status = VL53L4CD_ERROR_NONE;

  p_Version->major = VL53L4CD_IMPLEMENTATION_VER_MAJOR;
  p_Version->minor = VL53L4CD_IMPLEMENTATION_VER_MINOR;
  p_Version->build = VL53L4CD_IMPLEMENTATION_VER_BUILD;
  p_Version->revision = VL53L4CD_IMPLEMENTATION_VER_REVISION;
  return Status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_SetI2CAddress(
  uint8_t new_address)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status |= VL53L4CD_WrByte(dev, VL53L4CD_I2C_SLAVE_DEVICE_ADDRESS,
                            (uint8_t)(new_address >> (uint8_t)1));

  dev = new_address;
  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_GetSensorId(
  uint16_t *p_id)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status |= VL53L4CD_RdWord(dev, VL53L4CD_IDENTIFICATION_MODEL_ID, p_id);
  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_SensorInit()
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint8_t Addr, tmp;
  uint8_t continue_loop = 1;
  uint16_t i = 0;

  do {
    status |= VL53L4CD_RdByte(dev,
                              VL53L4CD_FIRMWARE_SYSTEM_STATUS, &tmp);

    if (tmp == (uint8_t)0x3) { /* Sensor booted */
      continue_loop = (uint8_t)0;
    } else if (i < (uint16_t)1000) {  /* Wait for boot */
      i++;
    } else { /* Timeout 1000ms reached */
      continue_loop = (uint8_t)0;
      status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
    }
    WaitMs(1);
  } while (continue_loop == (uint8_t)1);

  /* Load default configuration */
  for (Addr = (uint8_t)0x2D; Addr <= (uint8_t)0x87; Addr++) {
    status |= VL53L4CD_WrByte(dev, Addr,
                              VL53L4CD_DEFAULT_CONFIGURATION[
                             Addr - (uint8_t)0x2D]);
  }

  /* Start VHV */
  status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_START, (uint8_t)0x40);
  i  = (uint8_t)0;
  continue_loop = (uint8_t)1;
  do {
    status |= VL53L4CD_CheckForDataReady(&tmp);
    if (tmp == (uint8_t)1) { /* Data ready */
      continue_loop = (uint8_t)0;
    } else if (i < (uint16_t)1000) {  /* Wait for answer */
      i++;
    } else { /* Timeout 1000ms reached */
      continue_loop = (uint8_t)0;
      status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
    }
    WaitMs(1);
  } while (continue_loop == (uint8_t)1);

  status |= VL53L4CD_ClearInterrupt();
  status |= VL53L4CD_StopRanging();
  status |= VL53L4CD_WrByte(dev,
                            VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND,
                            (uint8_t)0x09);
  status |= VL53L4CD_WrByte(dev, 0x0B, (uint8_t)0);
  status |= VL53L4CD_WrWord(dev, 0x0024, 0x500);

  status |= VL53L4CD_SetRangeTiming(50, 0);

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_ClearInterrupt()
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_INTERRUPT_CLEAR, 0x01);
  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_StartRanging()
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint8_t data_ready, continue_loop = 1;
  uint16_t i = 0;
  uint32_t tmp;

  status |= VL53L4CD_RdDWord(dev, VL53L4CD_INTERMEASUREMENT_MS, &tmp);

  /* Sensor runs in continuous mode */
  if (tmp == (uint32_t)0) {
    status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_START, 0x21);
  }
  /* Sensor runs in autonomous mode */
  else {
    status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_START, 0x40);
  }

  do {
    status |= VL53L4CD_CheckForDataReady(&data_ready);
    if (data_ready == (uint8_t)1) { /* Data ready */
      continue_loop = (uint8_t)0;
    } else if (i < (uint16_t)1000) {  /* Wait for answer */
      i++;
    } else { /* Timeout 1000ms reached */
      continue_loop = (uint8_t)0;
      status = (uint8_t)VL53L4CD_ERROR_TIMEOUT;
    }
    WaitMs(1);
  } while (continue_loop == (uint8_t)1);

  status |= VL53L4CD_ClearInterrupt();

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_StopRanging()
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_START, 0x00);
  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_CheckForDataReady(
  uint8_t *p_is_data_ready)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint8_t temp;
  uint8_t int_pol;

  status |= VL53L4CD_RdByte(dev, VL53L4CD_GPIO_HV_MUX_CTRL, &temp);
  temp = temp & (uint8_t)0x10;
  temp = temp >> 4;

  if (temp == (uint8_t)1) {
    int_pol = (uint8_t)0;
  } else {
    int_pol = (uint8_t)1;
  }

  status |= VL53L4CD_RdByte(dev, VL53L4CD_GPIO_TIO_HV_STATUS, &temp);

  if ((temp & (uint8_t)1) == int_pol) {
    *p_is_data_ready = (uint8_t)1;
  } else {
    *p_is_data_ready = (uint8_t)0;
  }

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_SetRangeTiming(
  uint32_t timing_budget_ms,
  uint32_t inter_measurement_ms)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint16_t clock_pll, osc_frequency, ms_byte;
  uint32_t macro_period_us = 0, timing_budget_us = 0, ls_byte, tmp;
  float inter_measurement_factor = (float)1.055;

  status |= VL53L4CD_RdWord(dev, 0x0006, &osc_frequency);
  if (osc_frequency != (uint16_t)0) {
    timing_budget_us = timing_budget_ms * (uint32_t)1000;
    macro_period_us = (uint32_t)((uint32_t)2304 *
                                 ((uint32_t)0x40000000 / (uint32_t)osc_frequency)) >> 6;
  } else {
    status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
  }

  /* Timing budget check validity */
  if ((timing_budget_ms < (uint32_t)10)
      || (timing_budget_ms > (uint32_t)200) || (status != (uint8_t)0)) {
    status |= VL53L4CD_ERROR_INVALID_ARGUMENT;
  }
  /* Sensor runs in continuous mode */
  else if (inter_measurement_ms == (uint32_t)0) {
    status |= VL53L4CD_WrDWord(dev, VL53L4CD_INTERMEASUREMENT_MS, 0);
    timing_budget_us -= (uint32_t)2500;
  }
  /* Sensor runs in autonomous low power mode */
  else if (inter_measurement_ms > timing_budget_ms) {
    status |= VL53L4CD_RdWord(dev,
                              VL53L4CD_RESULT_OSC_CALIBRATE_VAL, &clock_pll);
    clock_pll = clock_pll & (uint16_t)0x3FF;
    inter_measurement_factor = inter_measurement_factor
                               * (float)inter_measurement_ms
                               * (float)clock_pll;
    status |= VL53L4CD_WrDWord(dev, VL53L4CD_INTERMEASUREMENT_MS,
                               (uint32_t)inter_measurement_factor);

    timing_budget_us -= (uint32_t)4300;
    timing_budget_us /= (uint32_t)2;

  }
  /* Invalid case */
  else {
    status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
  }

  if (status != (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT) {
    ms_byte = 0;
    timing_budget_us = timing_budget_us << 12;
    tmp = macro_period_us * (uint32_t)16;
    ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6))
              - (uint32_t)1;

    while ((ls_byte & 0xFFFFFF00U) > 0U) {
      ls_byte = ls_byte >> 1;
      ms_byte++;
    }
    ms_byte = (uint16_t)(ms_byte << 8)
              + (uint16_t)(ls_byte & (uint32_t)0xFF);
    status |= VL53L4CD_WrWord(dev, VL53L4CD_RANGE_CONFIG_A, ms_byte);

    ms_byte = 0;
    tmp = macro_period_us * (uint32_t)12;
    ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6))
              - (uint32_t)1;

    while ((ls_byte & 0xFFFFFF00U) > 0U) {
      ls_byte = ls_byte >> 1;
      ms_byte++;
    }
    ms_byte = (uint16_t)(ms_byte << 8)
              + (uint16_t)(ls_byte & (uint32_t)0xFF);
    status |= VL53L4CD_WrWord(dev, VL53L4CD_RANGE_CONFIG_B, ms_byte);
  }

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_GetRangeTiming(
  uint32_t *p_timing_budget_ms,
  uint32_t *p_inter_measurement_ms)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint16_t osc_frequency = 1, range_config_macrop_high, clock_pll = 1;
  uint32_t tmp, ls_byte, ms_byte, macro_period_us;
  float clock_pll_factor = (float)1.065;

  /* Get InterMeasurement */
  status |= VL53L4CD_RdDWord(dev, VL53L4CD_INTERMEASUREMENT_MS, &tmp);
  status |= VL53L4CD_RdWord(dev,
                            VL53L4CD_RESULT_OSC_CALIBRATE_VAL, &clock_pll);
  clock_pll = clock_pll & (uint16_t)0x3FF;
  clock_pll_factor = clock_pll_factor * (float)clock_pll;
  clock_pll = (uint16_t)clock_pll_factor;
  *p_inter_measurement_ms = (uint16_t)(tmp / (uint32_t)clock_pll);

  /* Get TimingBudget */
  status |= VL53L4CD_RdWord(dev, 0x0006, &osc_frequency);
  status |= VL53L4CD_RdWord(dev, VL53L4CD_RANGE_CONFIG_A,
                            &range_config_macrop_high);

  macro_period_us = (uint32_t)((uint32_t)2304 * ((uint32_t)0x40000000
                                                 / (uint32_t)osc_frequency)) >> 6;
  ls_byte = (range_config_macrop_high & (uint32_t)0x00FF) << 4;
  ms_byte = (range_config_macrop_high & (uint32_t)0xFF00) >> 8;
  ms_byte = (uint32_t)0x04 - (ms_byte - (uint32_t)1) - (uint32_t)1;

  macro_period_us = macro_period_us * (uint32_t)16;
  *p_timing_budget_ms = (((ls_byte + (uint32_t)1) * (macro_period_us >> 6))
                         - ((macro_period_us >> 6) >> 1)) >> 12;

  if (ms_byte < (uint8_t)12) {
    *p_timing_budget_ms = (uint32_t)(*p_timing_budget_ms
                                     >> (uint8_t)ms_byte);
  }

  /* Mode continuous */
  if (tmp == (uint32_t)0) {
    *p_timing_budget_ms += (uint32_t)2500;
  }
  /* Mode autonomous */
  else {
    *p_timing_budget_ms *= (uint32_t)2;
    *p_timing_budget_ms += (uint32_t)4300;
  }

  *p_timing_budget_ms = *p_timing_budget_ms / (uint32_t)1000;

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_GetResult(
  VL53L4CD_Result_t *p_result)
{
  // BEGIN MODIFIED CODE
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint16_t temp_16;

  // auto start = micros();
  // status |= VL53L4CD_RdByte(dev, VL53L4CD_RESULT_RANGE_STATUS,
  //                           &temp_8);
  // temp_8 = temp_8 & (uint8_t)0x1F;
  // if (temp_8 < (uint8_t)24) {
  //   temp_8 = status_rtn[temp_8];
  // }
  // p_result->range_status = temp_8;
  // Serial.printf("range_status: %d\n", micros() - start);

  // start = micros();
  // status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_SPAD_NB,
  //                           &temp_16);
  // p_result->number_of_spad = temp_16 / (uint16_t) 256;
  // Serial.printf("number_of_spad: %d\n", micros() - start);

  // start = micros();
  // status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_SIGNAL_RATE,
  //                           &temp_16);
  // p_result->signal_rate_kcps = temp_16 * (uint16_t) 8;
  // Serial.printf("signal_rate_kcps: %d\n", micros() - start);

  // start = micros();
  // status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_AMBIENT_RATE,
  //                           &temp_16);
  // p_result->ambient_rate_kcps = temp_16 * (uint16_t) 8;
  // Serial.printf("ambient_rate_kcps: %d\n", micros() - start);

  // start = micros();
  // status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_SIGMA,
  //                           &temp_16);
  // p_result->sigma_mm = temp_16 / (uint16_t) 4;
  // Serial.printf("sigma_mm: %d\n", micros() - start);

  status |= VL53L4CD_RdWord(dev, VL53L4CD_RESULT_DISTANCE,
                            &temp_16);
  p_result->distance_mm = temp_16;

  // p_result->signal_per_spad_kcps = p_result->signal_rate_kcps
  //                                  / p_result->number_of_spad;
  // p_result->ambient_per_spad_kcps = p_result->ambient_rate_kcps
  //                                   / p_result->number_of_spad;

  // END MODIFIED CODE
  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_SetOffset(
  int16_t OffsetValueInMm)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint16_t temp;

  temp = (uint16_t)((uint16_t)OffsetValueInMm * (uint16_t)4);

  status |= VL53L4CD_WrWord(dev, VL53L4CD_RANGE_OFFSET_MM, temp);
  status |= VL53L4CD_WrWord(dev, VL53L4CD_INNER_OFFSET_MM, (uint8_t)0x0);
  status |= VL53L4CD_WrWord(dev, VL53L4CD_OUTER_OFFSET_MM, (uint8_t)0x0);
  return status;
}

VL53L4CD_ERROR  VL53L4CD::VL53L4CD_GetOffset(
  int16_t *p_offset)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint16_t temp;

  status |= VL53L4CD_RdWord(dev, VL53L4CD_RANGE_OFFSET_MM, &temp);

  temp = temp << 3;
  temp = temp >> 5;
  *p_offset = (int16_t)(temp);

  if (*p_offset > 1024) {
    *p_offset = *p_offset - 2048;
  }

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_SetXtalk(
  uint16_t XtalkValueKcps)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status |= VL53L4CD_WrWord(dev,
                            VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS, 0x0000);
  status |= VL53L4CD_WrWord(dev,
                            VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS, 0x0000);
  status |= VL53L4CD_WrWord(dev,
                            VL53L4CD_XTALK_PLANE_OFFSET_KCPS,
                            (XtalkValueKcps << 9) / (uint16_t)1000);

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_GetXtalk(
  uint16_t *p_xtalk_kcps)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status |= VL53L4CD_RdWord(dev,
                            VL53L4CD_XTALK_PLANE_OFFSET_KCPS, p_xtalk_kcps);
  *p_xtalk_kcps = (uint16_t)(*p_xtalk_kcps * (uint16_t)1000) >> 9;

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_SetDetectionThresholds(
  uint16_t distance_low_mm,
  uint16_t distance_high_mm,
  uint8_t window)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_INTERRUPT, window);
  status |= VL53L4CD_WrWord(dev, VL53L4CD_THRESH_HIGH, distance_high_mm);
  status |= VL53L4CD_WrWord(dev, VL53L4CD_THRESH_LOW, distance_low_mm);
  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_GetDetectionThresholds(
  uint16_t *p_distance_low_mm,
  uint16_t *p_distance_high_mm,
  uint8_t *p_window)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status |= VL53L4CD_RdWord(dev, VL53L4CD_THRESH_HIGH, p_distance_high_mm);
  status |= VL53L4CD_RdWord(dev, VL53L4CD_THRESH_LOW, p_distance_low_mm);
  status |= VL53L4CD_RdByte(dev, VL53L4CD_SYSTEM_INTERRUPT, p_window);
  *p_window = (*p_window & (uint8_t)0x7);

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_SetSignalThreshold(
  uint16_t signal_kcps)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status |= VL53L4CD_WrWord(dev,
                            VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS, signal_kcps >> 3);
  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_GetSignalThreshold(
  uint16_t  *p_signal_kcps)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint16_t tmp = 0;

  status |= VL53L4CD_RdWord(dev,
                            VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
  *p_signal_kcps = tmp << 3;

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_SetSigmaThreshold(
  uint16_t  sigma_mm)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  if (sigma_mm > (uint16_t)((uint16_t)0xFFFF >> 2)) {
    status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
  } else {
    status |= VL53L4CD_WrWord(dev,
                              VL53L4CD_RANGE_CONFIG_SIGMA_THRESH, sigma_mm << 2);
  }

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_GetSigmaThreshold(
  uint16_t  *p_sigma_mm)
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;

  status += VL53L4CD_RdWord(dev,
                            VL53L4CD_RANGE_CONFIG_SIGMA_THRESH, p_sigma_mm);
  *p_sigma_mm = *p_sigma_mm >> 2;

  return status;
}

VL53L4CD_ERROR VL53L4CD::VL53L4CD_StartTemperatureUpdate()
{
  VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
  uint8_t tmp = 0, continue_loop = 1;
  uint16_t i = 0;

  status |= VL53L4CD_WrByte(dev,
                            VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, (uint8_t)0x81);
  status |= VL53L4CD_WrByte(dev, 0x0B, (uint8_t)0x92);
  status |= VL53L4CD_WrByte(dev, VL53L4CD_SYSTEM_START, (uint8_t)0x40);

  do {
    status |= VL53L4CD_CheckForDataReady(&tmp);
    if (tmp == (uint8_t)1) { /* Data ready */
      continue_loop = (uint8_t)0;
    } else if (i < (uint16_t)1000) {  /* Wait for answer */
      i++;
    } else { /* Timeout 1000ms reached */
      continue_loop = (uint8_t)0;
      status = (uint8_t)VL53L4CD_ERROR_TIMEOUT;
    }
    WaitMs(1);
  } while (continue_loop == (uint8_t)1);

  status |= VL53L4CD_ClearInterrupt();
  status |= VL53L4CD_StopRanging();

  status += VL53L4CD_WrByte(dev,
                            VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09);
  status += VL53L4CD_WrByte(dev, 0x0B, 0);
  return status;
}
