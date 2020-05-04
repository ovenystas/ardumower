/*
 * motor.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#include "Motor.h"
#include <Arduino.h>
#include "AdcManager.h"
#include "Drivers.h"
#include "Filter.h"

void Motor::config(float acceleration, int16_t pwmMax, int16_t powerMax,
    bool regulate, int16_t rpmMax, int16_t rpmSet)
{
  m_acceleration = acceleration;
  m_pwmMax = pwmMax;
  m_powerMax = powerMax;
  m_regulate = regulate;
  m_rpmMax = rpmMax;
  m_rpmSet = rpmSet;
}

void Motor::setRpmState()
{
  bool newRpmState = readRpmPin();
  if (newRpmState != m_rpmLastState)
  {
    m_rpmLastState = newRpmState;
    if (m_rpmLastState)
    {
      m_rpmCounter++;
    }
  }
}

uint16_t Motor::getSamplingTime()
{
  uint32_t curMillis = millis();
  uint32_t samplingTime = curMillis - m_lastSetSpeedTime;
  m_lastSetSpeedTime = curMillis;

  if (samplingTime > 1000)
  {
    samplingTime = 1;
  }
  return static_cast<uint16_t>(samplingTime);
}

bool Motor::isTimeTo(uint32_t* nextTime_p, uint16_t timeBetween)
{
  uint32_t curMillis = millis();
  if (curMillis >= *nextTime_p)
  {
    *nextTime_p = curMillis + timeBetween;
    return true;
  }
  return false;
}

