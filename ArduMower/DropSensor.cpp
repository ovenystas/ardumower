/*
 * DropSensor.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "DropSensor.h"

void DropSensor::setup(const uint8_t pin)
{
  m_pin = pin;
  m_detected = false;
  m_counter = 0;
  pinMode(pin, INPUT_PULLUP);
}

void DropSensor::check(DropSensor_Contact contactType)
{
  if (digitalRead(m_pin) == (int16_t)contactType)
  {
    m_detected = true;
    ++m_counter;
  }
}

void DropSensors::setup(const uint8_t* pins, DropSensor_Contact contactType,
    DropSensor* dropSensorArray_p, uint8_t len)
{
  m_use = false;
  m_contactType = contactType;
  m_len = len;
  m_lastRun = 0;
  m_dropSensorArray_p = dropSensorArray_p;

  for (uint8_t i = 0; i < len; i++)
  {
    dropSensorArray_p[i].setup(pins[i]);
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
