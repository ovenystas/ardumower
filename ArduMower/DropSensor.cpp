/*
 * DropSensor.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "DropSensor.h"

void DropSensor::check(DropSensor_Contact contactType)
{
  if (digitalRead(m_pin) == static_cast<int16_t>(contactType))
  {
    m_detected = true;
    ++m_counter;
  }
}

void DropSensors::check()
{
  m_lastRun = millis();
  for (uint8_t i = 0; i < m_len; i++)
  {
    m_dropSensorArray_p[i].check(m_contactType);
  }
}

void DropSensors::clearDetected()
{
  for (uint8_t i = 0; i < m_len; i++)
  {
    m_dropSensorArray_p[i].clearDetected();
  }
}
