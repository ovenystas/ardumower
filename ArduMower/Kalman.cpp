/*
 * Kalman.cpp
 *
 *  Created on: May 10, 2020
 *      Author: ove
 */

#include "Kalman.h"

// Kalman filter

// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Kalman::update(const float newAngle, const float newRate,
    const int16_t looptime, float x_angle)
{
  float dt = static_cast<float>(looptime) / 1000;
  x_angle += dt * (newRate - x_bias);
  P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
  P_01 += -dt * P_11;
  P_10 += -dt * P_11;
  P_11 += Q_gyro * dt;

  float y = newAngle - x_angle;
  float S = P_00 + R_angle;
  float K_0 = P_00 / S;
  float K_1 = P_10 / S;

  x_angle += K_0 * y;
  x_bias += K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return x_angle;
}
