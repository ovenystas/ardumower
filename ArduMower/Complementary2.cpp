/*
 * Complementary2.cpp
 *
 *  Created on: May 10, 2020
 *      Author: ove
 */

#include "Complementary2.h"

// Second-order complementary filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary2::update(const float newAngle, const float newRate,
    const int16_t looptime, float angle)
{
  float dt = static_cast<float>(looptime) / 1000.0f;

  float dAngle = newAngle - angle;
  float x1 = dAngle * 2 * k;
  float x2 = dAngle * k * k * dt;
  float y1 = newRate + x1 + x2;

  angle = dt * y1;

  return angle;
}
