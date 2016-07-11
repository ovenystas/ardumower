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
  filter.addValue(newAdcValue);
}
