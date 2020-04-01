/*
 * MotorMosFet.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include "MotorMosFet.h"
#include "AdcManager.h"
#include "Pid.h"

//#define Console Serial

void MotorMosFet::setup(void)
{
  pinMode(pinPwm, OUTPUT);
  pinMode(pinSense, INPUT);
  ADCMan.setCapture(pinSense, 1, true);
}

void MotorMosFet::setSpeed(void)
{
  setSpeed(m_pwmCur);
}

void MotorMosFet::setSpeed(const int16_t speed)
{
  uint8_t tmpSpeed = (uint8_t)constrain(speed, 0, m_pwmMax);
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
  FilterEmaI16_addValue(newAdcValue, &m_filter);
}

// Sets motor PWM
// - ensures that the motor is not switched to 100% too fast (acceleration)
// - ensures that the motor voltage is not higher than pwmMax
void MotorMosFet::control(void)
{
  int16_t pwmNew;

  if (m_regulate)
  {
    // Use PID regulator.
    float y = m_pid.compute(m_rpmMeas);
    pwmNew = (int)round(y);
  }
  else
  {
    // Direct control of PWM
    m_pwmSet = map(m_rpmSet, 0, m_rpmMax, 0, m_pwmMax);
    if (m_pwmSet < m_pwmCur)
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

  m_pwmCur = constrain(pwmNew, 0, m_pwmMax);
  setSpeed();
}
