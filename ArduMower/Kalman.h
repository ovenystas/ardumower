/*
 * Kalman.h
 *
 *  Created on: May 10, 2020
 *      Author: ove
 */

#pragma once

#include <Arduino.h>

class Kalman
{
public:
  float update(const float newAngle, const float newRate,
      const int16_t looptime, float x_angle);

private:
  const float Q_angle = 0.01f; //0.001
  const float Q_gyro = 0.0003f;  //0.003
  const float R_angle = 0.01f;  //0.03

  float x_bias {};
  float P_00 {};
  float P_01 {};
  float P_10 {};
  float P_11 {};
};
