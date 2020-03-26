/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat
 Copyright (c) 2014 by Maxime Carpentieri

 Private-use only! (you need to ask for a commercial-use)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Private-use only! (you need to ask for a commercial-use)

 How to find out P,I,D:
 1. Increase P until system starts to oscillate
 2. Set I =0.6 * P and D = 0.125 * P
*/

#include <Arduino.h>

#include "Pid.h"

void pid_setup(float Kp, float Ki, float Kd,
               float yMin, float yMax,
               float maxOutput, Pid* pid_p)
{
  pid_p->settings.Kp = Kp;
  pid_p->settings.Ki = Ki;
  pid_p->settings.Kd = Kd;
  pid_p->yMin = yMin;
  pid_p->yMax = yMax;
  pid_p->maxOutput = maxOutput;
  pid_p->errorOld = 0;
  pid_p->errorSum = 0;
  pid_p->setPoint = 0;
  pid_p->lastControlTime = micros();
}

float pid_compute(float processValue, Pid* pid_p)
{
  unsigned long now = micros();

  float dt = ((float)(now - pid_p->lastControlTime) / 1000000.0f);
  pid_p->lastControlTime = now;
  if (dt > 1.0)
  {
    dt = 1.0;   // should only happen for the very first call
  }

  // compute error
  float error = pid_p->setPoint - processValue;

  // integrate error
  pid_p->errorSum += error;

  // anti wind-up
  float iTerm = pid_p->settings.Ki * dt * pid_p->errorSum;
  if (iTerm < -pid_p->maxOutput)
  {
    iTerm = -pid_p->maxOutput;
    pid_p->errorSum = -pid_p->maxOutput / dt / pid_p->settings.Ki;
  }
  if (iTerm > pid_p->maxOutput)
  {
    iTerm = pid_p->maxOutput;
    pid_p->errorSum = pid_p->maxOutput / dt / pid_p->settings.Ki;
  }

  float out = pid_p->settings.Kp * error + iTerm +
      pid_p->settings.Kd / dt * (error - pid_p->errorOld);

  pid_p->errorOld = error;

  // restrict output to min/max
  out = constrain(out, pid_p->yMin, pid_p->yMax);

  return out;
}
