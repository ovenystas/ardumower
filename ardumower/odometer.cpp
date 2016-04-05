/*
 * odometer.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#include <Arduino.h>
#include "odometer.h"
#include "encoder.h"

void Odometer::setup(const int ticksPerRevolution,
                     const float ticksPerCm,
                     const float wheelBaseCm,
                     const uint8_t pin[2],
                     const uint8_t pin2[2],
                     const bool swapDir[2])
{
  this->ticksPerRevolution = ticksPerRevolution;
  this->ticksPerCm = ticksPerCm;
  this->wheelBaseCm = wheelBaseCm;

  encoder[LEFT].setup(pin[LEFT], pin2[LEFT], swapDir[LEFT]);
  encoder[RIGHT].setup(pin[RIGHT], pin2[RIGHT], swapDir[RIGHT]);
}

void Odometer::setup(const int ticksPerRevolution,
                     const float ticksPerCm,
                     const float wheelBaseCm,
                     const uint8_t pin[2],
                     const bool swapDir[2])
{
  this->ticksPerRevolution = ticksPerRevolution;
  this->ticksPerCm = ticksPerCm;
  this->wheelBaseCm = wheelBaseCm;

  encoder[LEFT].setup(pin[LEFT], swapDir[LEFT]);
  encoder[RIGHT].setup(pin[RIGHT], swapDir[RIGHT]);
}

void Odometer::read()
{
  encoder[LEFT].read();
  encoder[RIGHT].read();
}

void Odometer::setState(unsigned long timeMicros)
{
  encoder[LEFT].setState(timeMicros);
  encoder[RIGHT].setState(timeMicros);
}
