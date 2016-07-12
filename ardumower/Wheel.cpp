/*
 * wheel.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#include "Wheel.h"

enum dirE
{
  LEFT,
  RIGHT
};

bool Wheels::isTimeToRotationChange(void)
{
  unsigned long curMillis = millis();
  if (curMillis >= nextTimeRotationChange)
  {
    nextTimeRotationChange = curMillis + TIME_BETWEEN_ROTATION_CHANGE;
    return true;
  }
  return false;
}

void Wheels::control(void)
{
  //TODO: Optimize for speed by doing int math instead of float math.

  if (!wheel[Wheel::LEFT].motor.isTimeToControl())
  {
    return;
  }

  int16_t localSteer =
      (int16_t)(((double)wheel[Wheel::LEFT].motor.rpmMax) *
      (((double)steer) / 100.0));

  // If reversing, steer should also be reversed
  if (speed < 0)
  {
    localSteer = -localSteer;
  }

  int16_t rpmNew;
  int16_t rpmMax;
  int16_t rpmCurr;

  rpmCurr = wheel[Wheel::LEFT].encoder.getWheelRpmCurr();
  wheel[Wheel::LEFT].motor.setRpmMeas(rpmCurr);

  rpmMax = wheel[Wheel::LEFT].motor.rpmMax;
  rpmNew = (int16_t)((double)rpmMax * ((double)speed / 100.0)) - localSteer;
  rpmNew = constrain(rpmNew, -rpmMax, rpmMax);
  wheel[Wheel::LEFT].motor.setRpmSet(rpmNew);

  rpmCurr = wheel[Wheel::RIGHT].encoder.getWheelRpmCurr();
  wheel[Wheel::RIGHT].motor.setRpmMeas(rpmCurr);

  rpmMax = wheel[Wheel::RIGHT].motor.rpmMax;
  rpmNew = (int16_t)((double)rpmMax * ((double)speed / 100.0)) + localSteer;
  rpmNew = constrain(rpmNew, -rpmMax, rpmMax);
  wheel[Wheel::RIGHT].motor.setRpmSet(rpmNew);

  wheel[Wheel::LEFT].motor.control();
  wheel[Wheel::RIGHT].motor.control();
}
