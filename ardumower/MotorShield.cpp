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
  setSpeed((int)(pwmCur + 0.5));
}

void MotorShield::setSpeed(const int speed)
{
  int tmpSpeed = swapDir ? -speed : speed;
  digitalWrite(pinDir[channel], tmpSpeed < 0);
  analogWrite(pinPwm[channel], abs(tmpSpeed));
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

void MotorShield::setSpeed(const int speed, const bool brake)
{
  digitalWrite(pinBrake[channel], brake && speed == 0);
  setSpeed(speed);
}

void MotorShield::readCurrent(void)
{
  int16_t newAdcValue = ADCMan.read(pinSense[channel]);
  filter.addValue(newAdcValue);
}
