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

void MotorDrv::calcPower(float bat_V)
{
  m_powerMeas = static_cast<int16_t>(getAverageCurrentAsFloat() * bat_V / 1000 + 0.5);
}

void MotorDrv::calcPower(int16_t bat_mV)
{
  m_powerMeas = static_cast<int16_t>((getAverageCurrentAsFloat() * bat_mV) / 1000000L + 0.5);
}
