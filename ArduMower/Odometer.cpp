/*
 * odometer.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#include "Odometer.h"

void Odometer::setup(const int ticksPerRevolution,
                     const float ticksPerCm,
                     const float wheelBaseCm,
                     Encoder* encoderLeft_p,
                     Encoder* encoderRight_p,
                     Imu* imu_p)
{
  m_ticksPerRevolution = ticksPerRevolution;
  m_ticksPerCm = ticksPerCm;
  m_wheelBaseCm = wheelBaseCm;
  m_encoder.left_p = encoderLeft_p;
  m_encoder.right_p = encoderRight_p;
  m_imu_p = imu_p;
}

void Odometer::read(void)
{
  encoder_read(m_encoder.left_p);
  encoder_read(m_encoder.right_p);
}

// calculate map position by odometer sensors
void Odometer::calc(void)
{
  unsigned long curMillis = millis();

  int16_t odoLeft = encoder_getCounter(m_encoder.left_p);
  int16_t odoRight = encoder_getCounter(m_encoder.right_p);

  int16_t ticksLeft = odoLeft - m_lastOdoLeft;
  int16_t ticksRight = odoRight - m_lastOdoRight;

  m_lastOdoLeft = odoLeft;
  m_lastOdoRight = odoRight;

  float left_cm = (float)ticksLeft / m_ticksPerCm;
  float right_cm = (float)ticksRight / m_ticksPerCm;
  float avg_cm = (left_cm + right_cm) / 2.0;

  float wheelTheta = (left_cm - right_cm) / m_wheelBaseCm;
  float thetaOld = m_theta;
  m_theta += wheelTheta;

  float revolutionLeft = (float)ticksLeft / (float)m_ticksPerRevolution;
  float revolutionRight = (float)ticksRight / (float)m_ticksPerRevolution;

  float deltaTime = (float)(curMillis - m_lastWheelRpmTime) / 60000.0;

  float rpmLeft = revolutionLeft / deltaTime;
  float rpmRight = revolutionRight / deltaTime;

  encoder_setWheelRpmCurr(round(rpmLeft), m_encoder.left_p);
  encoder_setWheelRpmCurr(round(rpmRight), m_encoder.right_p);

  m_lastWheelRpmTime = curMillis;

  if (m_imu_p->m_use)
  {
    float yaw = m_imu_p->getYaw();
    m_x += avg_cm * sin(yaw);
    m_y += avg_cm * cos(yaw);
  }
  else
  {
    m_x += avg_cm * sin(thetaOld);
    m_y += avg_cm * cos(thetaOld);
  }
}
