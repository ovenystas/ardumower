/*
 * RainSensor.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "RainSensor.h"

void rainSensor_setup(const uint8_t pin, RainSensor* rainSensor_p)
{
  rainSensor_p->use = false;
  rainSensor_p->pin = pin;
  rainSensor_p->raining = false;
  rainSensor_p->counter = 0;
  pinMode(rainSensor_p->pin, INPUT);
}

void rainSensor_check(RainSensor* rainSensor_p)
{
  rainSensor_p->raining = (digitalRead(rainSensor_p->pin) == LOW);
  if (rainSensor_p->raining)
  {
    rainSensor_p->counter++;
  }
}
