/*
 * DropSensor.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include <Arduino.h>
#include "DropSensor.h"

void DropSensor::setup(const uint8_t pin, const boolean contactType)
{
  this->pin = pin;
  this->contactType = contactType;

  pinMode(pin, INPUT_PULLUP);
}

void DropSensor::simDetected()
{
  detected = true;
  counter++;
}

void DropSensor::check()
{
  if (digitalRead(pin) == contactType)
  {
    detected = true;
    counter++;
  }
}

void DropSensors::check()
{
  dropSensor[LEFT].check();
  dropSensor[RIGHT].check();
}

void DropSensors::clearDetected()
{
  dropSensor[LEFT].clearDetected();
  dropSensor[RIGHT].clearDetected();
}

bool DropSensors::isTimeToRun()
{
  unsigned long curMillis = millis();
  if (used && curMillis >= nextTime)
  {
    nextTime = curMillis + timeBetweenRuns;
    return true;
  }
  return false;
}
