/*
 * MotorDrv.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include "MotorDrv.h"
#include <Arduino.h>

int16_t MotorDrv::getSenseAdc(void)
{
  filter.getAverage();
}

int16_t MotorDrv::getCurrent(void)
{
  return (uint16_t((double)filter.getAverage() * scale + 0.5));
}

void MotorDrv::calcPower(float batV)
{
  powerMeas = (int16_t)(getCurrent() * batV / 1000 + 0.5);
}


void MotorDrv::setFilterAlpha(double alpha)
{
  filter.setAlpha(alpha);
}
