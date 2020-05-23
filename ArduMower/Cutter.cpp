/*
 * Cutter.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: ove
 */

#include <Arduino.h>
#include "Cutter.h"

void Cutter::control(void)
{
  m_motor.control();
}

void Cutter::turnOn()
{
  if (m_use)
  {
    int16_t rpmMax = m_motor.m_rpmMax;
    int16_t rpmNew = (static_cast<int32_t>(rpmMax) * m_idleRpm_percent) / 100;

    const int16_t rpmMin = 0; // MosFet driver does not support dir change.
    if (rpmMin < 0 && m_randomDir && random(1))
    {
      rpmNew = -rpmNew;
    }

    // TODO: Move constrain into setRpmSet
    rpmNew = constrain(rpmNew, rpmMin, rpmMax);
    m_motor.setRpmSet(rpmNew);

    m_isOn = true;
  }
}

void Cutter::turnOff()
{
  m_motor.setRpmSet(0);
  m_isOn = false;
}

void Cutter::toggleOnOff()
{
  if (m_isOn)
  {
    turnOff();
  }
  else
  {
    turnOn();
  }
}
