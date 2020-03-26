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
  int16_t rpmMax = motor.m_rpmMax;
  int16_t rpmNew = rpmMax * speed / 100;
  rpmNew = constrain(rpmNew, 0, rpmMax);
  motor.setRpmSet(rpmNew);
  motor.control();
}
