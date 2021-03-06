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
 2. Set I = 0.6 * P and D = 0.125 * P
*/

#include "Pid.h"

void Pid::setup(float Kp, float Ki, float Kd, float yMin, float yMax,
    float maxOutput)
{
  m_kp = Kp;
  m_ki = Ki;
  m_kd = Kd;
  m_yMin = yMin;
  m_yMax = yMax;
  m_maxOutput = maxOutput;

  m_errorOld = 0;
  m_errorSum = 0;
  m_setPoint = 0;
  m_lastControlTime = micros();
}

void Pid::setup(float Kp, float Ki, float Kd, float yMin, float yMax,
    float maxOutput, float regConstScale, float regConstMaxValue)
{
  m_settings.Kp.scale = m_settings.Ki.scale = m_settings.Kd.scale = regConstScale;
  m_settings.Kp.maxValue = m_settings.Ki.maxValue = m_settings.Kd.maxValue = regConstMaxValue;

  setup(Kp, Ki, Kd, yMin, yMax, maxOutput);
}

float Pid::compute(float processValue)
{
  unsigned long now = micros();

  float dt = ((float)(now - m_lastControlTime) / 1000000.0f);
  m_lastControlTime = now;

  if (dt > 1.0)
  {
    dt = 1.0; // Should only happen for the very first call
  }

  // Compute error
  float error = m_setPoint - processValue;

  // Integration error
  m_errorSum += error;

  // Anti wind-up
  float iTerm = m_ki * dt * m_errorSum;

  if (iTerm < -m_maxOutput)
  {
    iTerm = -m_maxOutput;
    m_errorSum = -m_maxOutput / dt / m_ki;
  }

  if (iTerm > m_maxOutput)
  {
    iTerm = m_maxOutput;
    m_errorSum = m_maxOutput / dt / m_ki;
  }

  float out = m_kp * error + iTerm + m_kd / dt * (error - m_errorOld);

  m_errorOld = error;

  // Restrict output
  out = constrain(out, m_yMin, m_yMax);

  return out;
}
