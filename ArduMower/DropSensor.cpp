/*
 * DropSensor.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "DropSensor.h"

void dropSensor_setup(const uint8_t pin, DropSensor* dropSensor_p)
{
  dropSensor_p->pin = pin;
  dropSensor_p->detected = false;
  dropSensor_p->counter = 0;
  pinMode(pin, INPUT_PULLUP);
}

void dropSensor_check(DropSensor* dropSensor_p, dropSensorContactE contactType)
{
  if (digitalRead(dropSensor_p->pin) == contactType)
  {
    dropSensor_p->detected = true;
    dropSensor_p->counter++;
  }
}
