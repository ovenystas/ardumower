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

#define FILTER_EMA_I32_ALPHA(x) ( (uint16_t)(x * UINT16_MAX) )
#define FILTER_EMA_I16_ALPHA(x) ( (uint8_t)(x * UINT8_MAX) )

typedef struct
{
  int32_t average;
  uint16_t alpha;
} FilterEmaI32;

typedef struct
{
  int16_t average;
  uint8_t alpha;
} FilterEmaI16;

static inline
void FilterEmaI32_setAlpha(double alpha, FilterEmaI32* filter_p)
{
  filter_p->alpha = FILTER_EMA_I32_ALPHA(alpha);
}

static inline
void FilterEmaI32_init(double alpha, FilterEmaI32* filter_p)
{
  filter_p->alpha = FILTER_EMA_I32_ALPHA(alpha);
  filter_p->average = 0;
}

void FilterEmaI32_addValue(int32_t in, FilterEmaI32* filter_p);

static inline
int32_t FilterEmaI32_getAverage(const FilterEmaI32* filter_p)
{
  return filter_p->average;
}

static inline
void FilterEmaI16_setAlpha(double alpha, FilterEmaI16* filter_p)
{
  filter_p->alpha = FILTER_EMA_I16_ALPHA(alpha);
}

static inline
void FilterEmaI16_init(double alpha, FilterEmaI16* filter_p)
{
  filter_p->alpha = FILTER_EMA_I16_ALPHA(alpha);
  filter_p->average = 0;
}

void FilterEmaI16_addValue(int16_t in, FilterEmaI16* filter_p);

static inline
int16_t FilterEmaI16_getAverage(const FilterEmaI16* filter_p)
{
  return filter_p->average;
}
