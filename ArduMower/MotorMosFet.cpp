/*
 * MotorMosFet.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include "MotorMosFet.h"
#include "AdcManager.h"
#include "Pid.h"

void MotorMosFet::setup()
{
  pinMode(m_pinPwm, OUTPUT);
  pinMode(m_pinSense, INPUT);
  ADCMan.setCapture(m_pinSense, 1, true);
}

void MotorMosFet::setSpeed()
{
  setSpeed(m_pwmCur);
}

void MotorMosFet::setSpeed(int16_t speed)
{
  uint8_t tmpSpeed = (uint8_t)constrain(speed, 0, m_pwmMax);
  analogWrite(m_pinPwm, tmpSpeed);
}

void MotorMosFet::setSpeed(int16_t speed, bool brake)
{
  (void)brake; // No brake available
  setSpeed(speed);
}

void MotorMosFet::readCurrent()
{
  int16_t newAdcValue = ADCMan.read(m_pinSense);
  m_filter.addValue(newAdcValue);
}

// Sets motor PWM
// - ensures that the motor is not switched to 100% too fast (acceleration)
// - ensures that the motor voltage is not higher than pwmMax
void MotorMosFet::control()
{
  int16_t pwmNew;

  if (m_regulate)
  {
    // Use PID regulator.
    float y = m_pid.compute(m_rpmMeas);
    pwmNew = static_cast<int>(round(y));
  }
  else
  {
    // Direct control of PWM
    m_pwmSet = static_cast<int16_t>(map(m_rpmSet, 0, m_rpmMax, 0, m_pwmMax));
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
      pwmNew = m_pwmCur +
          getSamplingTime() * static_cast<float>(m_pwmSet - m_pwmCur) /
          m_acceleration;
    }
  }

  m_pwmCur = constrain(pwmNew, 0, m_pwmMax);
  setSpeed();
}
