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

class FilterEmaI32
{
  public:
    void setAlpha(double alpha)
    {
      this->alpha = FILTER_EMA_I32_ALPHA(alpha);
    }
    void addValue(int32_t in);
    const int32_t getAverage(void) const
    {
      return average;
    }

  private:
    int32_t average {};
    uint16_t alpha {FILTER_EMA_I32_ALPHA(1.0)};
};

class FilterEmaI16
{
  public:
    void setAlpha(double alpha)
    {
      this->alpha = FILTER_EMA_I16_ALPHA(alpha);
    }
    void addValue(int16_t in);
    const int16_t getAverage(void) const
    {
      return average;
    }

  private:
    int16_t average {};
    uint8_t alpha {FILTER_EMA_I16_ALPHA(1.0)};
};

#endif /* FILTER_H */
