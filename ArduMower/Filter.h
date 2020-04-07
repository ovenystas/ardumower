/*
 * filter.h
 *
 *  Created on: Apr 17, 2016
 *      Author: ove
 *
 * The exponential moving average is a type of IIR filter that is easy to
 * implement in C and uses minimal resources. Unlike a simple moving average,
 * it does not require a RAM buffer to store previous samples. It just has to
 * store one value (the previous average).
 *
 * An exponential moving average is expressed as the following equation:
 *   avg[n] = (in * alpha) + avg[n-1]*(1-alpha).
 *
 * Implementing this equation using floating point math is straightforward but
 * using fixed point variables is a little tricky. The code snippet here use
 * 32-bit signed integers for the average and input values. Intermediate values
 * need to use 64-bit math to avoid overflow errors.
 *
 * Alpha values close to zero represent heavy averaging while an alpha value of
 * one has no averaging.
 */
#pragma once

#include <Arduino.h>

#define FILTER_EMA_I32_ALPHA(x) ( static_cast<uint16_t>(x * UINT16_MAX) )
#define FILTER_EMA_I16_ALPHA(x) ( static_cast<uint8_t>(x * UINT8_MAX) )

#define FILTER_EMA_I32_ALPHA_REVERSE(x) ( (static_cast<float>(x) / UINT16_MAX) )
#define FILTER_EMA_I16_ALPHA_REVERSE(x) ( (static_cast<float>(x) / UINT8_MAX) )


class FilterEmaI32
{
public:
  FilterEmaI32(float alpha) :
    m_alpha(FILTER_EMA_I32_ALPHA(alpha)) {};

  void addValue(int32_t in);

  int32_t getAverage()
  {
    return m_average;
  }

  void setAlpha(float alpha)
  {
    m_alpha = FILTER_EMA_I32_ALPHA(alpha);
  }

  float getAlpha()
  {
    return FILTER_EMA_I32_ALPHA_REVERSE(m_alpha);
  }

private:
  int32_t m_average {};
  uint16_t m_alpha;
};


class FilterEmaI16
{
public:
  FilterEmaI16(float alpha) :
    m_alpha(FILTER_EMA_I16_ALPHA(alpha)) {};

  void addValue(int16_t in);

  int16_t getAverage()
  {
    return m_average;
  }

  void setAlpha(float alpha)
  {
    m_alpha = FILTER_EMA_I16_ALPHA(alpha);
  }

  float getAlpha()
  {
    return FILTER_EMA_I16_ALPHA_REVERSE(m_alpha);
  }

private:
  int16_t m_average {};
  uint8_t m_alpha;
};
