/*
 * wheel.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#include "Wheel.h"

bool Wheels::isTimeToRotationChange()
{
  unsigned long curMillis = millis();
  if (curMillis >= nextTimeRotationChange)
  {
    nextTimeRotationChange = curMillis + timeBetweenRotationChange;
    return true;
  }
  return false;
}
