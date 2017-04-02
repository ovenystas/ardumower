/*
 * MotorMosFet.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include <Arduino.h>
#include "MotorMosFet.h"
#include "AdcManager.h"

//#define Console Serial

void MotorMosFet::setup(void)
{
  pinMode(pinPwm, OUTPUT);
  pinMode(pinSense, INPUT);
  ADCMan.setCapture(pinSense, 1, true);
}

void MotorMosFet::setSpeed(void)
{
  setSpeed(pwmCur);
}

void MotorMosFet::setSpeed(const int16_t speed)
{
  uint8_t tmpSpeed = (uint8_t)constrain(speed, 0, pwmMax);
  analogWrite(pinPwm, tmpSpeed);
//  Console.print("MotorMosFet::setSpeed pinPwm=");
//  Console.print(pinPwm);
//  Console.print(" speed=");
//  Console.println(tmpSpeed);
}

void MotorMosFet::setSpeed(const int16_t speed, const bool brake)
{
  (void)brake; // No brake available
  setSpeed(speed);
}

void MotorMosFet::readCurrent(void)
{
  int16_t newAdcValue = ADCMan.read(pinSense);
  FilterEmaI16_addValue(newAdcValue, &filter);
}

// Sets motor PWM
// - ensures that the motor is not switched to 100% too fast (acceleration)
// - ensures that the motor voltage is not higher than pwmMax
void MotorMosFet::control(void)
{
  int16_t pwmNew;

  if (regulate)
  {
    // Use PID regulator.
    pid.setYMin(0);
    pid.setYMax(pwmMax);
    pid.setMaxOutput(pwmMax);
    pid.setSetpoint(rpmSet);
    float y = pid.compute(rpmMeas);
    pwmNew = (int)round(y);
  }
  else
  {
    // Direct control of PWM
    pwmSet = map(rpmSet, 0, rpmMax, 0, pwmMax);
    if (pwmSet < pwmCur)
    {
      // Ignore acceleration if speed is lowered (e.g. motor is shut down).
      pwmNew = pwmSet;
    }
    else
    {
      // Use acceleration when speed is increased
      // http://phrogz.net/js/framerate-independent-low-pass-filter.html
      // smoothed += elapsedTime * ( newValue - smoothed ) / smoothing;
      int16_t addPwm = getSamplingTime() * (float)(pwmSet - pwmCur) / acceleration;
      pwmNew = pwmCur + addPwm;
    }
  }

  pwmCur = constrain(pwmNew, 0, pwmMax);
  setSpeed();
}
