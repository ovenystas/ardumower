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
#include "pid.h"

void PID::setup(const float Kp, const float Ki, const float Kd)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::setup(const float Kp, const float Ki, const float Kd,
                const float y_min, const float y_max,
                const float max_output)
{
  setup(Kp, Kd, Ki);
  this->y_min = y_min;
  this->y_max = y_max;
  this->max_output = max_output;
}

float PID::compute()
{
  compute(this->x);
}

float PID::compute(float x)
{
  unsigned long now = micros();

  float Ta = ((now - lastControlTime) / 1000000.0);
  lastControlTime = now;
  if (Ta > 1.0)
  {
    Ta = 1.0;   // should only happen for the very first call
  }

  // compute error
  float e = (w - x);

  // integrate error
  esum += e;

  // anti wind-up
  float iTerm = Ki * Ta * esum;
  if (iTerm < -max_output)
  {
    iTerm = -max_output;
    esum = -max_output / Ta / Ki;
  }
  if (iTerm > max_output)
  {
    iTerm = max_output;
    esum = max_output / Ta / Ki;
  }
  y = Kp * e + iTerm + Kd / Ta * (e - eold);
  eold = e;

  // restrict output to min/max
  if (y > y_max)
  {
    y = y_max;
  }
  if (y < y_min)
  {
    y = y_min;
  }

  return y;
}
