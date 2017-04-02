/*
 * Bumper.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "Bumper.h"

void bumper_setup(const uint8_t pin, Bumper* bumper_p)
{
  bumper_p->pin = pin;
  bumper_p->hit = false;
  bumper_p->counter = 0;
  pinMode(bumper_p->pin, INPUT_PULLUP);
}

void bumper_check(Bumper* bumper_p)
{
  if (digitalRead(bumper_p->pin) == LOW)
  {
    bumper_p->hit = true;
    bumper_p->counter++;
  }
}

bool bumpers_isAnyHit(const Bumpers* bumpers_p)
{
  for (uint8_t i = 0; i < bumpers_p->len; i++)
  {
    if (bumper_isHit(&bumpers_p->bumperArray_p[i]))
    {
      return true;
    }
  }
  return false;
}
