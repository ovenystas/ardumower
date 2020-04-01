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

#include "Pid.h"

void Pid::setup(float Kp, float Ki, float Kd, float yMin, float yMax,
    float maxOutput)
{
  m_settings.Kp = Kp;
  m_settings.Ki = Ki;
  m_settings.Kd = Kd;
  m_yMin = yMin;
  m_yMax = yMax;
  m_maxOutput = maxOutput;
  m_errorOld = 0;
  m_errorSum = 0;
  m_setPoint = 0;
  m_lastControlTime = micros();
}

float Pid::compute(float processValue)
{
  unsigned long now = micros();

  float dt = ((float)(now - m_lastControlTime) / 1000000.0f);
  m_lastControlTime = now;

  if (dt > 1.0)
  {
    dt = 1.0;   // should only happen for the very first call
  }

  // compute error
  float error = m_setPoint - processValue;

  // integrate error
  m_errorSum += error;

  // anti wind-up
  float iTerm = m_settings.Ki * dt * m_errorSum;
  if (iTerm < -m_maxOutput)
  {
    iTerm = -m_maxOutput;
    m_errorSum = -m_maxOutput / dt / m_settings.Ki;
  }
  if (iTerm > m_maxOutput)
  {
    iTerm = m_maxOutput;
    m_errorSum = m_maxOutput / dt / m_settings.Ki;
  }

  float out = m_settings.Kp * error + iTerm +
      m_settings.Kd / dt * (error - m_errorOld);

  m_errorOld = error;

  // restrict output to min/max
  out = constrain(out, m_yMin, m_yMax);

  return out;
}
