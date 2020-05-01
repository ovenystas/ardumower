/*
 * wheel.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#include "Wheel.h"

void Wheel::control(int8_t speed)
{
  m_motor.setRpmMeas(m_encoder.getWheelRpmCurr());

  m_motor.setRpmSet((m_motor.m_rpmMax * speed) / 100);

  m_motor.control();
}

bool Wheels::isTimeToRotationChange(void)
{
  uint32_t curMillis = millis();
  if (curMillis >= m_nextTimeRotationChange)
  {
    m_nextTimeRotationChange = curMillis + TIME_BETWEEN_ROTATION_CHANGE_ms;
    return true;
  }
  return false;
}

void Wheels::control(void)
{
  int8_t localSteer = 0;

  // If reversing, steer should also be reversed
  if (m_speed < 0)
  {
    localSteer = -m_steer;
  }
  else
  {
    localSteer = m_steer;
  }

  int16_t leftSpeed  = static_cast<int16_t>(m_speed) - localSteer;
  int16_t rightSpeed = static_cast<int16_t>(m_speed) + localSteer;

  leftSpeed  = constrain(leftSpeed,  -100, 100);
  rightSpeed = constrain(rightSpeed, -100, 100);

  m_wheel[Wheel::LEFT].control(static_cast<int8_t>(leftSpeed));
  m_wheel[Wheel::RIGHT].control(static_cast<int8_t>(rightSpeed));
}
