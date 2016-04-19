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
                     const uint8_t pin[2],
                     const uint8_t pin2[2],
                     const bool swapDir[2])
{
  this->ticksPerRevolution = ticksPerRevolution;
  this->ticksPerCm = ticksPerCm;
  this->wheelBaseCm = wheelBaseCm;

  encoder[LEFT].setup(pin[LEFT], pin2[LEFT], swapDir[LEFT]);
  encoder[RIGHT].setup(pin[RIGHT], pin2[RIGHT], swapDir[RIGHT]);
}

void Odometer::read()
{
  encoder[LEFT].read();
  encoder[RIGHT].read();
}

void Odometer::setState(unsigned long timeMicros)
{
  encoder[LEFT].setState(timeMicros);
  encoder[RIGHT].setState(timeMicros);
}

// calculate map position by odometer sensors
void Odometer::calc(const Imu &imu)
{
  unsigned long curMillis = millis();
  if (!use || curMillis < nextTime)
  {
    return;
  }
  nextTime = curMillis + 300;

  static int lastOdoLeft = 0;
  static int lastOdoRight = 0;

  int odoLeft = encoder[Odometer::LEFT].counter;
  int odoRight = encoder[Odometer::RIGHT].counter;

  int ticksLeft = odoLeft - lastOdoLeft;
  int ticksRight = odoRight - lastOdoRight;

  lastOdoLeft = odoLeft;
  lastOdoRight = odoRight;

  float left_cm = (float)ticksLeft / ticksPerCm;
  float right_cm = (float)ticksRight / ticksPerCm;
  float avg_cm = (left_cm + right_cm) / 2.0;
  float wheel_theta = (left_cm - right_cm) / wheelBaseCm;
  float thetaOld = theta;
  theta += wheel_theta;

  encoder[Odometer::LEFT].wheelRpmCurr =
      double((((float)ticksLeft / (float)ticksPerRevolution) /
              (float)(millis() - lastWheelRpmTime)) * 60000.0);

  encoder[Odometer::RIGHT].wheelRpmCurr =
      double((((float)ticksRight / (float)ticksPerRevolution) /
              (float)(millis() - lastWheelRpmTime)) * 60000.0);

  lastWheelRpmTime = millis();

  if (imu.use)
  {
    x += avg_cm * sin(imu.ypr.yaw);
    y += avg_cm * cos(imu.ypr.yaw);
  }
  else
  {
    x += avg_cm * sin(thetaOld);
    y += avg_cm * cos(thetaOld);
  }
}
