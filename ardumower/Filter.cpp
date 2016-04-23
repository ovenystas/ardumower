/*
 * filter.cpp
 *
 *  Created on: Apr 17, 2016
 *      Author: ove
 */

#include "Filter.h"

void FilterEmaI32::addValue(int32_t in)
{
  int64_t tmp0; //calculations must be done in 64-bit math to avoid overflow
  tmp0 = (int64_t)in * (alpha) + (int64_t)average * (65536 - alpha);
  average = (int32_t)((tmp0 + 32768) / 65536); //scale back to 32-bit (with rounding)
}

void FilterEmaI16::addValue(int16_t in)
{
  int32_t tmp0; //calculations must be done in 32-bit math to avoid overflow
  tmp0 = (int32_t)in * (alpha) + (int32_t)average * (256 - alpha);
  average = (int16_t)((tmp0 + 128) / 256); //scale back to 16-bit (with rounding)
}
