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
#include "filter.h"

void Motor::setup(const float acceleration, const int pwmMax, const float powerMax,
                  const bool regulate, const int rpmMax, const int rpmSet,
                  const float senseScale, const uint8_t pinDir,
                  const uint8_t pinPwm, const uint8_t pinSense,
                  const uint8_t pinRpm, const uint8_t pinBrake)
{
  this->acceleration = acceleration;
  this->pwmMax = pwmMax;
  this->powerMax = powerMax;
  this->regulate = regulate;
  this->rpmMax = rpmMax;
  this->rpmSet = rpmSet;
  this->senseScale = senseScale;
  this->pinDir = pinDir;
  this->pinPwm = pinPwm;
  this->pinSense = pinSense;
  this->pinRpm = pinRpm;
  this->pinBrake = pinBrake;
  pinMode(pinDir, OUTPUT);
  digitalWrite(pinDir, LOW);
  pinMode(pinPwm, OUTPUT);
  digitalWrite(pinDir, LOW);
  pinMode(pinSense, INPUT);
  pinMode(pinRpm, INPUT);
  pinMode(pinBrake, OUTPUT);
  digitalWrite(pinDir, LOW);
}

bool Motor::readRpmPin()
{
  return digitalRead(pinRpm);
}

int Motor::readSensePin()
{
  senseAdc = ADCMan.read(pinSense);
  return senseAdc;
}

float Motor::calcCurrent(double alpha)
{
  // Exponential Moving Average (EMA) filter (ardumower version)
  currentMeas = currentMeas * (1.0 - alpha) + (double)senseAdc * senseScale * alpha;
  // Exponential Moving Average (EMA) filter (Ove's version)
  //senseCurrent = dsp_ema_i16((double)senseAdc * senseScale, senseCurrent, DSP_EMA_I16_ALPHA(0.05));
  return currentMeas;
}

float Motor::calcPower(float batV)
{
  powerMeas = currentMeas * batV / 1000;
  return powerMeas;
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

void Motor::setSpeed()
{
  int speed;
  if (swapDir)
  {
    speed = -pwmCur;
  }
  else
  {
    speed = pwmCur;
  }
  setArdumoto(pinDir, pinPwm, speed);
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
