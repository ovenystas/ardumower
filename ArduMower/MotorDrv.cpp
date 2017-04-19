/*
 * MotorDrv.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include "MotorDrv.h"
#include <Arduino.h>

int16_t MotorDrv::getAverageCurrent(void)
{
  return (uint16_t((double)FilterEmaI16_getAverage(&filter) * scale + 0.5));
}

void MotorDrv::calcPower(float batV)
{
  powerMeas = (int16_t)(getAverageCurrent() * batV / 1000 + 0.5);
}
