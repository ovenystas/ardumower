/*
 * Button.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "Button.h"
#include <Arduino.h>

void Button::setup(const uint8_t pin)
{
  this->pin = pin;
  pinMode(pin, INPUT_PULLUP);
}

bool Button::isPressed(void)
{
  return (digitalRead(pin) == LOW);
}

bool Button::isTimeToCheck()
{
  unsigned long curMillis = millis();
  if (use && curMillis >= nextTimeCheck)
  {
    nextTimeCheck = curMillis + timeBetweenChecks;
    return true;
  }
  return false;
}

bool Button::isTimeToRun()
{
  unsigned long curMillis = millis();
  if (use && curMillis >= nextTime)
  {
    nextTime = curMillis + timeBetweenRuns;
    return true;
  }
  return false;
}
