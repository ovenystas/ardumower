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

void Wheel::control(int8_t speed)
{
  int16_t rpmCurr = encoder.getWheelRpmCurr();
  motor.setRpmMeas(rpmCurr);

  int16_t rpmNew = motor.rpmMax * speed / 100;
  motor.setRpmSet(rpmNew);

  motor.control();
}

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
  if (!wheel[Wheel::LEFT].motor.isTimeToControl())
  {
    return;
  }

  int8_t localSteer;

  // If reversing, steer should also be reversed
  if (speed < 0)
  {
    localSteer = -steer;
  }
  else
  {
    localSteer = steer;
  }

  int16_t leftSpeed  = (int16_t)speed - localSteer;
  int16_t rightSpeed = (int16_t)speed + localSteer;

  leftSpeed  = constrain(leftSpeed,  -100, 100);
  rightSpeed = constrain(rightSpeed, -100, 100);

  wheel[Wheel::LEFT].control((int8_t)leftSpeed);
  wheel[Wheel::RIGHT].control((int8_t)rightSpeed);
}
