/*
 * Complementary2.h
 *
 *  Created on: May 10, 2020
 *      Author: ove
 */

#pragma once

#include <Arduino.h>

class Complementary2
{
public:
  float update(const float newAngle, const float newRate,
      const int16_t looptime, float angle);

private:
  const float k = 10;
};
