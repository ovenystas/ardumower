/*
 * lawnSensor.cpp
 *
 *  Created on: Apr 3, 2016
 *      Author: ove
 */

#include "LawnSensor.h"

void LawnSensor::setup(uint8_t sendPin, uint8_t receivePin)
{
  m_sendPin = sendPin;
  m_receivePin = receivePin;

  pinMode(m_sendPin, OUTPUT);
  pinMode(m_receivePin, INPUT);

  m_value = 0;
  m_valueOld = 0;
}


uint16_t LawnSensor::measureLawnCapacity()
{
  digitalWrite(m_sendPin, HIGH);

  // TODO: Improve this
  uint16_t t = 0;
  while (digitalRead(m_receivePin) == LOW)
  {
    t++;
  }

  digitalWrite(m_sendPin, LOW);

  return t;
}

void LawnSensor::read()
{
  const float accel = 0.03f;

  m_value = (1.0f - accel) * m_value +
      accel * (float)measureLawnCapacity();
}

void LawnSensors::setup(const uint8_t* sendPins, const uint8_t* receivePins,
    LawnSensor* lawnSensorArray_p, const uint8_t len)
{
  m_use = false;
  m_len = len;
  m_lawnSensorArray_p = lawnSensorArray_p;

  for (uint8_t i = 0; i < len; i++)
  {
    lawnSensorArray_p[i].setup(sendPins[i], receivePins[i]);
  }

  m_detected = false;
  m_counter = 0;
}

void LawnSensors::check()
{
  LawnSensor* sensorF_p = &m_lawnSensorArray_p[0];
  LawnSensor* sensorB_p = &m_lawnSensorArray_p[1];

  float deltaF = (sensorF_p->m_value / sensorF_p->m_valueOld) * 100.0f;
  float deltaB = (sensorB_p->m_value / sensorB_p->m_valueOld) * 100.0f;

  if (deltaF <= 95 || deltaB <= 95)
  {
    m_counter++;
    m_detected = true;
  }

  sensorF_p->m_valueOld = sensorF_p->m_value;
  sensorB_p->m_valueOld = sensorB_p->m_value;
}
