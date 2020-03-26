/*
 * MotorShield.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include <Arduino.h>
#include "MotorShield.h"
#include "AdcManager.h"

//#define Console Serial

void MotorShield::setup(void)
{
  pinMode(pinDir[channel], OUTPUT);
  digitalWrite(pinDir[channel], LOW);

  pinMode(pinPwm[channel], OUTPUT);
  digitalWrite(pinDir[channel], LOW);

  pinMode(pinBrake[channel], OUTPUT);
  digitalWrite(pinBrake[channel], LOW);

  pinMode(pinSense[channel], INPUT);
  ADCMan.setCapture(pinSense[channel], 1, true);
}

void MotorShield::setSpeed(void)
{
  setSpeed(pwmCur);
}

void MotorShield::setSpeed(const int16_t speed)
{
  int16_t tmpSpeed = swapDir ? -speed : speed;
  digitalWrite(pinDir[channel], tmpSpeed < 0);
  analogWrite(pinPwm[channel], constrain(abs(tmpSpeed), 0, pwmMax));
//  Console.print("MotorShield::setSpeed ch=");
//  Console.print(channel);
//  Console.print(" pinDir=");
//  Console.print(pinDir[channel]);
//  Console.print(" pinPwm=");
//  Console.print(pinPwm[channel]);
//  Console.print(" dir=");
//  Console.print(tmpSpeed < 0);
//  Console.print(" speed=");
//  Console.println(tmpSpeed);
}

void MotorShield::setSpeed(const int16_t speed, const bool brake)
{
  digitalWrite(pinBrake[channel], brake && speed == 0);
  setSpeed(speed);
}

void MotorShield::readCurrent(void)
{
  int16_t newAdcValue = ADCMan.read(pinSense[channel]);
  FilterEmaI16_addValue(newAdcValue, &filter);
}

// Sets motor PWM
// - ensures that the motor is not switched to 100% too fast (acceleration)
// - ensures that the motor voltage is not higher than pwmMax
void MotorShield::control(void)
{
  int pwmNew;

  if (regulate)
  {
    // Use PID regulator.
    float y = pid_compute(rpmMeas, &pid);
    pwmNew = (int)(round(y));
  }
  else
  {
    // Direct control of PWM
    pwmSet = map(rpmSet, -rpmMax, rpmMax, -pwmMax, pwmMax);
    if (pwmSet <= pwmCur)
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

  pwmCur = constrain(pwmNew, -pwmMax, pwmMax);
  setSpeed();
}
