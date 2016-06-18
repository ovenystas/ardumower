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
  setSpeed((int16_t)(pwmCur + 0.5));
}

void MotorMosFet::setSpeed(const int16_t speed)
{
  int16_t tmpSpeed = speed < 0 ? 0 : speed;
  analogWrite(pinPwm, tmpSpeed);
//  Console.print("MotorMosFet::setSpeed pinPwm=");
//  Console.print(pinPwm);
//  Console.print(" speed=");
//  Console.println(tmpSpeed);
}

void MotorMosFet::setSpeed(const int16_t speed, const bool brake)
{
  (void)brake;
  setSpeed(speed);
}

void MotorMosFet::readCurrent(void)
{
  int16_t newAdcValue = ADCMan.read(pinSense);
  filter.addValue(newAdcValue);
}
