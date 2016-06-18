/*
 * wheel.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#include "Wheel.h"

bool Wheels::isTimeToRotationChange(void)
{
  unsigned long curMillis = millis();
  if (curMillis >= nextTimeRotationChange)
  {
    nextTimeRotationChange = curMillis + TIME_BETWEEN_ROTATION_CHANGE;
    return true;
  }
  return false;
}
