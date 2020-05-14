/*
 * odometer.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#include "Odometer.h"

void Odometer::read(void)
{
  m_encoder.left.read();
  m_encoder.right.read();
}

// calculate map position by odometer sensors
void Odometer::calc(void)
{
  uint32_t curMillis = millis();

  int16_t odoLeft = m_encoder.left.getCounter();
  int16_t odoRight = m_encoder.right.getCounter();

  int16_t ticksLeft = odoLeft - m_lastOdoLeft;
  int16_t ticksRight = odoRight - m_lastOdoRight;

  m_lastOdoLeft = odoLeft;
  m_lastOdoRight = odoRight;

  float left_cm = static_cast<float>(ticksLeft) / m_ticksPerCm;
  float right_cm = static_cast<float>(ticksRight) / m_ticksPerCm;
  float avg_cm = (left_cm + right_cm) / 2.0;

  float wheelTheta = (left_cm - right_cm) / m_wheelBaseCm;
  float thetaOld = m_theta;
  m_theta += wheelTheta;

  float revolutionLeft =
      static_cast<float>(ticksLeft) / static_cast<float>(m_ticksPerRevolution);
  float revolutionRight =
      static_cast<float>(ticksRight) / static_cast<float>(m_ticksPerRevolution);

  float deltaTime = static_cast<float>(curMillis - m_lastWheelRpmTime) / 60000.0;

  float rpmLeft = revolutionLeft / deltaTime;
  float rpmRight = revolutionRight / deltaTime;

  m_encoder.left.setWheelRpmCurr(round(rpmLeft));
  m_encoder.right.setWheelRpmCurr(round(rpmRight));

  m_lastWheelRpmTime = curMillis;

  if (m_imu.isUsed())
  {
    float yaw = m_imu.getYaw();
    m_x += avg_cm * sin(yaw);
    m_y += avg_cm * cos(yaw);
  }
  else
  {
    m_x += avg_cm * sin(thetaOld);
    m_y += avg_cm * cos(thetaOld);
  }
}
