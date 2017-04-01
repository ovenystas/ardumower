/*
 * Button.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "Button.h"
#include <Arduino.h>
#include "Config.h"

#define TIME_BETWEEN_RUNS 1000

static uint8_t counter = 0;
bool button_use = true;

void button_setup(void)
{
  pinMode(PIN_BUTTON, INPUT_PULLUP);
}

bool button_isPressed(void)
{
  return (digitalRead(PIN_BUTTON) == LOW);
}

bool button_isTimeToRun(void)
{
  static unsigned long nextTime = 0;
  unsigned long curMillis = millis();
  if (button_use && curMillis >= nextTime)
  {
    nextTime = curMillis + TIME_BETWEEN_RUNS;
    return true;
  }
  return false;
}

uint8_t button_getCounter(void)
{
  return counter;
}

void button_incCounter(void)
{
  counter++;
}

void button_clearCounter(void)
{
  counter = 0;
}
