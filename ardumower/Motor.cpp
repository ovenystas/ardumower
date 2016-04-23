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

void Motor::config(const float acceleration, const int pwmMax, const int powerMax,
                   const bool regulate, const int rpmMax, const int rpmSet)
{
  this->acceleration = acceleration;
  this->pwmMax = pwmMax;
  this->powerMax = powerMax;
  this->regulate = regulate;
  this->rpmMax = rpmMax;
  this->rpmSet = rpmSet;
}

bool Motor::readRpmPin()
{
  return digitalRead(pinRpm);
}

void Motor::setRpmState()
{
  boolean newRpmState = readRpmPin();
  if (newRpmState != rpmLastState)
  {
    rpmLastState = newRpmState;
    if (rpmLastState)
    {
      rpmCounter++;
    }
  }
}

unsigned long Motor::getSamplingTime()
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
    nextTimeRpmMeas = curMillis + timeBetweenRpmMeas;
    lastRpmTime = curMillis;
    return true;
  }
  return false;
}

bool Motor::isTimeTo(unsigned long* nextTime_p, const unsigned int timeBetween)
{
  unsigned long curMillis = millis();
  if (curMillis >= *nextTime_p)
  {
    *nextTime_p = curMillis + timeBetween;
    return true;
  }
  return false;
}

bool Motor::isTimeToControl()
{
  return isTimeTo(&nextTimeControl, timeBetweenControl);
}

bool Motor::isTimeToCheckPower()
{
  return isTimeTo(&nextTimeCheckPower, timeBetweenCheckPower);
}

bool Motor::isTimeToReadSensor()
{
  return isTimeTo(&nextTimeReadSensor, timeBetweenReadSensor);
}

bool Motor::isWaitAfterStuckEnd()
{
  return (millis() >= lastTimeStucked + timeWaitAfterStuck);
}

void Motor::gotStuck()
{
  lastTimeStucked = millis();
}
