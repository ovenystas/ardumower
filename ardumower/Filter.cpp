/*
 * filter.cpp
 *
 *  Created on: Apr 17, 2016
 *      Author: ove
 */

#include "Filter.h"

void FilterEmaI32_addValue(int32_t in, FilterEmaI32* filter_p)
{
   //calculations must be done in 64-bit math to avoid overflow
   int64_t tmp0 = (int64_t)in * (filter_p->alpha) +
                  (int64_t)filter_p->average * (65536 - filter_p->alpha);
  //scale back to 32-bit (with rounding)
  filter_p->average = (int32_t)((tmp0 + 32768) / 65536);
}

void FilterEmaI16_addValue(int16_t in, FilterEmaI16* filter_p)
{
   //calculations must be done in 32-bit math to avoid overflow
   int32_t tmp0 = (int32_t)in * (filter_p->alpha) +
                  (int32_t)filter_p->average * (256 - filter_p->alpha);
  //scale back to 16-bit (with rounding)
  filter_p->average = (int16_t)((tmp0 + 128) / 256);
}
