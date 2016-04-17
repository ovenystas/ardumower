/*
 * filter.cpp
 *
 *  Created on: Apr 17, 2016
 *      Author: ove
 */

#include "filter.h"

int32_t dsp_ema_i32(int32_t in, int32_t average, uint16_t alpha)
{
  int64_t tmp0; //calcs must be done in 64-bit math to avoid overflow
  tmp0 = (int64_t)in * (alpha) + (int64_t)average * (65536 - alpha);
  return (int32_t)((tmp0 + 32768) / 65536); //scale back to 32-bit (with rounding)
}

int16_t dsp_ema_i16(int16_t in, int16_t average, uint8_t alpha)
{
  int32_t tmp0; //calcs must be done in 32-bit math to avoid overflow
  tmp0 = (int32_t)in * (alpha) + (int32_t)average * (256 - alpha);
  return (int16_t)((tmp0 + 128) / 256); //scale back to 16-bit (with rounding)
}
