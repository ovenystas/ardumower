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
  // TODO: Optimize speed by using int math instead of double
  int16_t rpmMax = motor.pwmMax;
  int16_t rpmNew = (double)rpmMax * ((double)speed / 100.0);
  rpmNew = constrain(rpmNew, 0, rpmMax);
  motor.setRpmSet(rpmNew);
  motor.control();
}
