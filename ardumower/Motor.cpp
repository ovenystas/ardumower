/*
 * motor.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: ove
 */

#include "Motor.h"
#include <Arduino.h>
#include "AdcManager.h"
#include "drivers.h"
#include "Filter.h"

void Motor::config(const float acceleration, const int16_t pwmMax,
                   const int16_t powerMax, const bool regulate,
                   const int16_t rpmMax, const int16_t rpmSet)
{
  this->acceleration = acceleration;
  this->pwmMax = pwmMax;
  this->powerMax = powerMax;
  this->regulate = regulate;
  this->rpmMax = rpmMax;
  this->rpmSet = rpmSet;
}

void Motor::setRpmState(void)
{
  bool newRpmState = readRpmPin();
  if (newRpmState != rpmLastState)
  {
    rpmLastState = newRpmState;
    if (rpmLastState)
    {
      rpmCounter++;
    }
  }
}

uint16_t Motor::getSamplingTime(void)
{
  uint32_t curMillis = millis();
  uint32_t samplingTime = curMillis - lastSetSpeedTime;
  lastSetSpeedTime = curMillis;

  if (samplingTime > 1000)
  {
    samplingTime = 1;
  }
  return (uint16_t)samplingTime;
}

bool Motor::isTimeForRpmMeas(unsigned long* timeSinceLast_p)
{
  uint32_t curMillis = millis();
  *timeSinceLast_p = curMillis - lastRpmTime;
  if (curMillis >= nextTimeRpmMeas)
  {
    nextTimeRpmMeas = curMillis + TIME_BETWEEN_RPM_MEAS;
    lastRpmTime = curMillis;
    return true;
  }
  return false;
}

bool Motor::isTimeTo(uint32_t* nextTime_p, const uint16_t timeBetween)
{
  uint32_t curMillis = millis();
  if (curMillis >= *nextTime_p)
  {
    *nextTime_p = curMillis + timeBetween;
    return true;
  }
  return false;
}

