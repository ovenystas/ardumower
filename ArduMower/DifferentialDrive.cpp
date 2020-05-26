/*
 * DifferentialDrive.cpp
 *
 *  Created on: May 25, 2020
 *      Author: ove
 */

#include "DifferentialDrive.h"

bool DifferentialDrive::isTimeToRotationChange(void)
{
  uint32_t curMillis = millis();
  if (curMillis - m_lastTimeRotationChange >= TIME_BETWEEN_ROTATION_CHANGE_ms)
  {
    m_lastTimeRotationChange = curMillis;
    return true;
  }
  return false;
}

void DifferentialDrive::control(void)
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

  m_wheelLeft.control(static_cast<int8_t>(leftSpeed));
  m_wheelRight.control(static_cast<int8_t>(rightSpeed));
}

Wheel* DifferentialDrive::getWheel(SideE side)
{
  if (side == SideE::LEFT)
  {
    return &m_wheelLeft;
  }
  else
  {
    return &m_wheelRight;
  }
}

void DifferentialDrive::toggleRotateDir()
{
  if (m_rotateDir == LEFT)
  {
    m_rotateDir = RIGHT;
  }
  else
  {
    m_rotateDir = LEFT;
  }
}
