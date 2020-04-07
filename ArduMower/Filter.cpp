/*
 * filter.cpp
 *
 *  Created on: Apr 17, 2016
 *      Author: ove
 */

#include "Filter.h"

void FilterEmaI32::addValue(int32_t in)
{
  //calculations must be done in 64-bit math to avoid overflow
  int64_t tmp = static_cast<int64_t>(in) * (m_alpha) +
      static_cast<int64_t>(m_average) * (UINT16_MAX - m_alpha);

  //scale back to 32-bit (with rounding)
  int16_t halfVal = tmp < 0 ? -INT16_MAX : INT16_MAX;
  m_average = static_cast<int32_t>((tmp + halfVal) / UINT16_MAX);
}

void FilterEmaI16::addValue(int16_t in)
{
  //calculations must be done in 32-bit math to avoid overflow
  int32_t tmp = static_cast<int32_t>(in) * (m_alpha) +
      static_cast<int32_t>(m_average) * (UINT8_MAX - m_alpha);

  //scale back to 16-bit (with rounding)
  int8_t halfVal = tmp < 0 ? -INT8_MAX : INT8_MAX;
  m_average = static_cast<int16_t>((tmp + halfVal) / UINT8_MAX);
}
