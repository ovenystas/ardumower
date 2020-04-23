/*
 * RainSensor.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "RainSensor.h"

void RainSensor::setup(const uint8_t pin)
{
  m_use = false;
  m_pin = pin;
  m_raining = false;
  m_counter = 0;
  pinMode(m_pin, INPUT);
}

void RainSensor::check()
{
  m_raining = (digitalRead(m_pin) == LOW);
  if (m_raining)
  {
    m_counter++;
  }
}
