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

void Wheels::stop(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = 0;
  wheel[Wheel::RIGHT].motor.rpmSet = 0;
}

void Wheels::slowDown(void)
{
  wheel[Wheel::LEFT].motor.rpmSet /= 1.5;
  wheel[Wheel::RIGHT].motor.rpmSet /= 1.5;
}

void Wheels::speedUp(void)
{
  wheel[Wheel::LEFT].motor.rpmSet *= 1.5;
  wheel[Wheel::RIGHT].motor.rpmSet *= 1.5;
}

void Wheels::forwardFullSpeed(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = wheel[Wheel::LEFT].motor.rpmMax;
  wheel[Wheel::RIGHT].motor.rpmSet = wheel[Wheel::RIGHT].motor.rpmMax;
}

void Wheels::forwardHalfSpeed(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = wheel[Wheel::LEFT].motor.rpmHalf;
  wheel[Wheel::RIGHT].motor.rpmSet = wheel[Wheel::RIGHT].motor.rpmHalf;
}

void Wheels::reverseFullSpeed(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = -wheel[Wheel::LEFT].motor.rpmMax;
  wheel[Wheel::RIGHT].motor.rpmSet = -wheel[Wheel::RIGHT].motor.rpmMax;
}

void Wheels::reverseFastSpeed(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = -wheel[Wheel::LEFT].motor.rpmFast;
  wheel[Wheel::RIGHT].motor.rpmSet = -wheel[Wheel::RIGHT].motor.rpmFast;
}

void Wheels::reverseSlowSpeed(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = -wheel[Wheel::LEFT].motor.rpmSlow;
  wheel[Wheel::RIGHT].motor.rpmSet = -wheel[Wheel::RIGHT].motor.rpmSlow;
}

void Wheels::rollFullRight(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = wheel[Wheel::LEFT].motor.rpmMax;
  wheel[Wheel::RIGHT].motor.rpmSet = -wheel[Wheel::RIGHT].motor.rpmMax;
}

void Wheels::rollFullLeft(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = -wheel[Wheel::LEFT].motor.rpmMax;
  wheel[Wheel::RIGHT].motor.rpmSet = wheel[Wheel::RIGHT].motor.rpmMax;
}

void Wheels::rollFull(bool dir)
{
  if (dir == LEFT)
  {
    rollFullLeft();
  }
  else
  {
    rollFullRight();
  }
}

void Wheels::rollFastRight(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = wheel[Wheel::LEFT].motor.rpmFast;
  wheel[Wheel::RIGHT].motor.rpmSet = -wheel[Wheel::RIGHT].motor.rpmFast;
}

void Wheels::rollFastLeft(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = -wheel[Wheel::LEFT].motor.rpmFast;
  wheel[Wheel::RIGHT].motor.rpmSet = wheel[Wheel::RIGHT].motor.rpmFast;
}

void Wheels::rollFast(bool dir)
{
  if (dir == LEFT)
  {
    rollFastLeft();
  }
  else
  {
    rollFastRight();
  }
}

void Wheels::rollHalfRight(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = wheel[Wheel::LEFT].motor.rpmHalf;
  wheel[Wheel::RIGHT].motor.rpmSet = -wheel[Wheel::RIGHT].motor.rpmHalf;
}

void Wheels::rollHalfLeft(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = -wheel[Wheel::LEFT].motor.rpmHalf;
  wheel[Wheel::RIGHT].motor.rpmSet = wheel[Wheel::RIGHT].motor.rpmHalf;
}

void Wheels::rollHalf(bool dir)
{
  if (dir == LEFT)
  {
    rollHalfLeft();
  }
  else
  {
    rollHalfRight();
  }
}

void Wheels::rollSlowRight(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = wheel[Wheel::LEFT].motor.rpmSlow;
  wheel[Wheel::RIGHT].motor.rpmSet = -wheel[Wheel::RIGHT].motor.rpmSlow;
}

void Wheels::rollSlowLeft(void)
{
  wheel[Wheel::LEFT].motor.rpmSet = -wheel[Wheel::LEFT].motor.rpmSlow;
  wheel[Wheel::RIGHT].motor.rpmSet = wheel[Wheel::RIGHT].motor.rpmSlow;
}

void Wheels::rollSlow(bool dir)
{
  if (dir == LEFT)
  {
    rollSlowLeft();
  }
  else
  {
    rollSlowRight();
  }
}
