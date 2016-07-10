/*
 * Bumper.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "Bumper.h"

void Bumper::setup(const uint8_t pin)
{
  this->pin = pin;
  pinMode(pin, INPUT_PULLUP);
}

void Bumper::simHit(void)
{
  hit = true;
  counter++;
}

void Bumper::check(void)
{
  if (digitalRead(pin) == LOW)
  {
    hit = true;
    counter++;
  }
}

void Bumpers::check(void)
{
  bumper[LEFT].check();
  bumper[RIGHT].check();
}

void Bumpers::clearHit(void)
{
  bumper[LEFT].clearHit();
  bumper[RIGHT].clearHit();
}

bool Bumpers::isAnyHit(void)
{
  return bumper[LEFT].isHit() || bumper[RIGHT].isHit();
}

bool Bumpers::isTimeToRun(void)
{
  unsigned long curMillis = millis();
  if (used && curMillis >= nextTime)
  {
    nextTime = curMillis + TIME_BETWEEN_RUNS;
    return true;
  }
  return false;
}
