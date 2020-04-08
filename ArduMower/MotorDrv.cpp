/*
 * MotorDrv.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include "MotorDrv.h"

int16_t MotorDrv::getAverageCurrent()
{
  return static_cast<int16_t>(getAverageCurrentAsFloat() + 0.5);
}

void MotorDrv::calcPower(float batV)
{
  m_powerMeas = static_cast<int16_t>(getAverageCurrentAsFloat() * batV / 1000 + 0.5);
}
