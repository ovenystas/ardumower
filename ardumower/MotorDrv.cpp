/*
 * MotorDrv.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ove
 */

#include "MotorDrv.h"
#include <Arduino.h>

int16_t MotorDrv::getAverageCurrent(void)
{
  return (uint16_t((double)filter.getAverage() * scale + 0.5));
}

void MotorDrv::calcPower(float batV)
{
  powerMeas = (int16_t)(getAverageCurrent() * batV / 1000 + 0.5);
}

// Sets motor PWM
// - ensures that the motor is not switched to 100% too fast (acceleration)
// - ensures that the motor voltage is not higher than pwmMax
void MotorDrv::control(void)
{
  int pwmNew;

  if (regulate)
  {
    // Use PID regulator.
    pid.setSetpoint(rpmSet);
    pwmNew = (int)(pid.compute(rpmMeas) + 0.5);
  }
  else
  {
    if (pwmSet < pwmCur)
    {
      // Ignore acceleration if speed is lowered (e.g. motor is shut down).
      pwmNew = pwmSet;
    }
    else
    {
      // http://phrogz.net/js/framerate-independent-low-pass-filter.html
      // smoothed += elapsedTime * ( newValue - smoothed ) / smoothing;
      int addPwm = getSamplingTime() * (float)(pwmSet - pwmCur) / acceleration;
      pwmNew = pwmCur + addPwm;
    }
  }
  pwmCur = constrain(pwmNew, 0, pwmMax);
  setSpeed();
}
