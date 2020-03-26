/*
 * Button.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "Button.h"

#define TIME_BETWEEN_RUNS 1000

void button_setup(uint8_t pin, Button* button_p)
{
  button_p->use = true;
  button_p->pin = pin;
  button_p->counter = 0;
  button_p->lastRun = 0;
  pinMode(pin, INPUT_PULLUP);
}

bool button_isTimeToRun(Button* button_p)
{
  if (button_p->use)
  {
    unsigned long curMillis = millis();
    if (curMillis - button_p->lastRun >= TIME_BETWEEN_RUNS)
    {
      button_p->lastRun = curMillis;
      return true;
    }
  }
  return false;
}
