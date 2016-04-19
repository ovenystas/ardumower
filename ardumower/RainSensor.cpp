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

void RainSensor::read()
{
  raining = (digitalRead(pin) == LOW);
}

void RainSensor::check()
{
  read();
  if (raining)
  {
    counter++;
  }
}

bool RainSensor::isTimeToRun()
{
  unsigned long curMillis = millis();
  if (used && curMillis >= nextTime)
  {
    nextTime = curMillis + timeBetweenRuns;
    return true;
  }
  return false;
}
