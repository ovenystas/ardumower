/*
 * Button.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "Button.h"

#define TIME_BETWEEN_RUNS 1000

bool Button::isTimeToRun()
{
  if (m_use)
  {
    unsigned long curMillis = millis();
    if (curMillis - m_lastRun >= TIME_BETWEEN_RUNS)
    {
      m_lastRun = curMillis;
      return true;
    }
  }
  return false;
}
