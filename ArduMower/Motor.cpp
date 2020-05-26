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
  uint16_t samplingTime = static_cast<uint16_t>(curMillis - m_lastTimeSetSpeed);
  m_lastTimeSetSpeed = curMillis;

  if (samplingTime > 1000)
  {
    samplingTime = 1;
  }
  return samplingTime;
}

bool Motor::isTimeTo(uint32_t& lastTime, uint16_t timeBetween)
{
  uint32_t curMillis = millis();
  if (curMillis - lastTime >= timeBetween)
  {
    lastTime = curMillis;
    return true;
  }
  return false;
}

int16_t Motor::getAverageCurrent()
{
  return static_cast<int16_t>(getAverageCurrentAsFloat() + 0.5);
}

void Motor::calcPower(float bat_V)
{
  m_powerMeas = static_cast<int16_t>(getAverageCurrentAsFloat() * bat_V / 1000 + 0.5);
}

void Motor::calcPower(int16_t bat_mV)
{
  m_powerMeas = static_cast<int16_t>((getAverageCurrentAsFloat() * bat_mV) / 1000000L + 0.5);
}
