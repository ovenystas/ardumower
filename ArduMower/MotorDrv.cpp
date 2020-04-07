/*
 * MotorDrv.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include "MotorDrv.h"

int16_t MotorDrv::getAverageCurrent()
{
  return (uint16_t((float)m_filter.getAverage() * m_scale + 0.5));
}

void MotorDrv::calcPower(float batV)
{
  m_powerMeas = (int16_t)(getAverageCurrent() * batV / 1000 + 0.5);
}
