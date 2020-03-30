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
  int16_t rpmMax = m_motor.m_rpmMax;
  int16_t rpmNew = rpmMax * m_speed / 100;
  rpmNew = constrain(rpmNew, 0, rpmMax);
  m_motor.setRpmSet(rpmNew);
  m_motor.control();
}
