/*
 * rainSensor.h
 *
 *  Created on: Apr 7, 2016
 *      Author: ove
 */

#ifndef RAINSENSOR_H
#define RAINSENSOR_H

#include <Arduino.h>

typedef struct
{
  bool use;
  uint8_t pin;
  bool raining;
  uint16_t counter;
} RainSensor;

void rainSensor_setup(const uint8_t pin, RainSensor* rainSensor_p);
void rainSensor_check(RainSensor* rainSensor_p);

static inline
bool rainSensor_isRaining(RainSensor* rainSensor_p)
{
  return rainSensor_p->raining;
}

static inline
uint16_t rainSensor_getCounter(RainSensor* rainSensor_p)
{
  return rainSensor_p->counter;
}

#endif /* RAINSENSOR_H */
