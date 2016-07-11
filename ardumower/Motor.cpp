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

void Motor::updateRpms(void)
{
  rpmFast = (int)(((float)rpmMax / RPM_DIVISOR_FAST) + 0.5);
  rpmHalf = (int)(((float)rpmMax / RPM_DIVISOR_HALF) + 0.5);
  rpmSlow = (int)(((float)rpmMax / RPM_DIVISOR_SLOW) + 0.5);
}

void Motor::config(const float acceleration, const int pwmMax, const int powerMax,
                   const bool regulate, const int rpmMax, const int rpmSet)
{
  this->acceleration = acceleration;
  this->pwmMax = pwmMax;
  this->powerMax = powerMax;
  this->regulate = regulate;
  this->rpmMax = rpmMax;
  updateRpms();
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

unsigned long Motor::getSamplingTime(void)
{
  unsigned long curMillis = millis();
  unsigned long samplingTime = curMillis - lastSetSpeedTime;
  lastSetSpeedTime = curMillis;

  if (samplingTime > 1000)
  {
    samplingTime = 1;
  }
  return samplingTime;
}

bool Motor::isTimeForRpmMeas(unsigned long* timeSinceLast_p)
{
  unsigned long curMillis = millis();
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
  unsigned long curMillis = millis();
  if (curMillis >= *nextTime_p)
  {
    *nextTime_p = curMillis + timeBetween;
    return true;
  }
  return false;
}

