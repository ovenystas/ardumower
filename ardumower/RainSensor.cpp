/*
 * RainSensor.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include <Arduino.h>
#include "RainSensor.h"

void RainSensor::setup(const uint8_t pin)
{
  this->pin = pin;
  pinMode(pin, INPUT);
}

void RainSensor::read(void)
{
  raining = (digitalRead(pin) == LOW);
}

void RainSensor::check(void)
{
  read();
  if (raining)
  {
    counter++;
  }
}

bool RainSensor::isTimeToRun(void)
{
  unsigned long curMillis = millis();
  if (used && curMillis >= nextTime)
  {
    nextTime = curMillis + TIME_BETWEEN_RUNS;
    return true;
  }
  return false;
}
