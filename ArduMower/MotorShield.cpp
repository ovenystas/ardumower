/*
 * MotorShield.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include "MotorShield.h"
#include "AdcManager.h"

//#define Console Serial

void MotorShield::setup(void)
{
  pinMode(pinDir[m_channel], OUTPUT);
  digitalWrite(pinDir[m_channel], LOW);

  pinMode(pinPwm[m_channel], OUTPUT);
  digitalWrite(pinDir[m_channel], LOW);

  pinMode(pinBrake[m_channel], OUTPUT);
  digitalWrite(pinBrake[m_channel], LOW);

  pinMode(pinSense[m_channel], INPUT);
  ADCMan.setCapture(pinSense[m_channel], 1, true);
}

void MotorShield::setSpeed(void)
{
  setSpeed(m_pwmCur);
}

void MotorShield::setSpeed(const int16_t speed)
{
  int16_t tmpSpeed = m_swapDir ? -speed : speed;
  digitalWrite(pinDir[m_channel], tmpSpeed < 0);
  analogWrite(pinPwm[m_channel], constrain(abs(tmpSpeed), 0, m_pwmMax));
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
  digitalWrite(pinBrake[m_channel], brake && speed == 0);
  setSpeed(speed);
}

void MotorShield::readCurrent(void)
{
  int16_t newAdcValue = ADCMan.read(pinSense[m_channel]);
  FilterEmaI16_addValue(newAdcValue, &m_filter);
}

// Sets motor PWM
// - ensures that the motor is not switched to 100% too fast (acceleration)
// - ensures that the motor voltage is not higher than pwmMax
void MotorShield::control(void)
{
  int pwmNew;

  if (m_regulate)
  {
    // Use PID regulator.
    float y = m_pid.compute(m_rpmMeas);
    pwmNew = (int)(round(y));
  }
  else
  {
    // Direct control of PWM
    m_pwmSet = map(m_rpmSet, -m_rpmMax, m_rpmMax, -m_pwmMax, m_pwmMax);
    if (m_pwmSet <= m_pwmCur)
    {
      // Ignore acceleration if speed is lowered (e.g. motor is shut down).
      pwmNew = m_pwmSet;
    }
    else
    {
      // Use acceleration when speed is increased
      // http://phrogz.net/js/framerate-independent-low-pass-filter.html
      // smoothed += elapsedTime * ( newValue - smoothed ) / smoothing;
      int16_t addPwm = getSamplingTime() * (float)(m_pwmSet - m_pwmCur) / m_acceleration;
      pwmNew = m_pwmCur + addPwm;
    }
  }

  m_pwmCur = constrain(pwmNew, -m_pwmMax, m_pwmMax);
  setSpeed();
}
