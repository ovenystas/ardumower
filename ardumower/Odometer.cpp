/*
 * odometer.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#include <Arduino.h>

#include "Odometer.h"

void Odometer::setup(const int ticksPerRevolution,
                     const float ticksPerCm,
                     const float wheelBaseCm,
                     Encoder* encoderLeft_p,
                     Encoder* encoderRight_p,
                     Imu* imu_p)
{
  this->ticksPerRevolution = ticksPerRevolution;
  this->ticksPerCm = ticksPerCm;
  this->wheelBaseCm = wheelBaseCm;
  this->encoder.left_p = encoderLeft_p;
  this->encoder.right_p = encoderRight_p;
  this->imu_p = imu_p;
}

void Odometer::loop(void)
{
  unsigned long curMillis = millis();
  if (!use || curMillis < nextTime)
  {
    return;
  }
  nextTime = curMillis + TIME_BETWEEN_CALCS;

  calc();
}

void Odometer::readAndSetState(void)
{
  encoder.left_p->read();
  encoder.right_p->read();

  encoder.left_p->setState();
  encoder.right_p->setState();
}

// calculate map position by odometer sensors
void Odometer::calc(void)
{
  unsigned long curMillis = millis();

  int16_t odoLeft = encoder.left_p->getCounter();
  int16_t odoRight = encoder.right_p->getCounter();

  int16_t ticksLeft = odoLeft - lastOdoLeft;
  int16_t ticksRight = odoRight - lastOdoRight;

  lastOdoLeft = odoLeft;
  lastOdoRight = odoRight;

  float left_cm = (float)ticksLeft / ticksPerCm;
  float right_cm = (float)ticksRight / ticksPerCm;
  float avg_cm = (left_cm + right_cm) / 2.0;
  float wheelTheta = (left_cm - right_cm) / wheelBaseCm;
  float thetaOld = theta;
  theta += wheelTheta;

  float revolutionLeft = (float)ticksLeft / (float)ticksPerRevolution;
  float revolutionRight = (float)ticksRight / (float)ticksPerRevolution;

  float deltaTime = (float)(curMillis - lastWheelRpmTime) / 60000.0;

  encoder.left_p->setWheelRpmCurr(revolutionLeft / deltaTime);
  encoder.right_p->setWheelRpmCurr(revolutionRight / deltaTime);

  lastWheelRpmTime = curMillis;

  if (imu_p->use)
  {
    float yaw = imu_p->getYaw();
    x += avg_cm * sin(yaw);
    y += avg_cm * cos(yaw);
  }
  else
  {
    x += avg_cm * sin(thetaOld);
    y += avg_cm * cos(thetaOld);
  }
}
