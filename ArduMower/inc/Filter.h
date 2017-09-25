/*
 * filter.h
 *
 *  Created on: Apr 17, 2016
 *      Author: ove
 */

#ifndef FILTER_H
#define FILTER_H

#include <Arduino.h>

#define FILTER_EMA_I32_ALPHA(x) ( (uint16_t)(x * 65535) )
#define FILTER_EMA_I16_ALPHA(x) ( (uint8_t)(x * 255) )

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

void FilterEmaI16_addValue(int16_t in, FilterEmaI16* filter_p);

static inline
int16_t FilterEmaI16_getAverage(const FilterEmaI16* filter_p)
{
  return filter_p->average;
}

#endif /* FILTER_H */
