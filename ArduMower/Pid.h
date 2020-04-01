/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat

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
 */
#pragma once

#include <Arduino.h>

/*
 digital PID controller
 */

struct Pid_settingsT
{
  float Kp;         // proportional control
  float Ki;         // integral control
  float Kd;         // differential control
};

class Pid
{
public:
  Pid() = default;
  Pid(float Kp, float Ki, float Kd, float yMin, float yMax, float maxOutput)
  {
    setup(Kp, Ki, Kd, yMin, yMax, maxOutput);
  }
  void setup(float Kp, float Ki, float Kd, float yMin, float yMax,
      float maxOutput);

  float compute(float processValue);

  float getSetPoint()
  {
    return m_setPoint;
  }

  void setSetPoint(float setPoint)
  {
    m_setPoint = setPoint;
  }

  Pid_settingsT* getSettings()
  {
    return &m_settings;
  }

  void setSettings(Pid_settingsT* settings_p)
  {
    m_settings = *settings_p;
  }

  float getErrorOld()
  {
    return m_errorOld;
  }

private:
  Pid_settingsT m_settings {};
  float m_setPoint {};
  float m_yMin {};
  float m_yMax {};
  float m_maxOutput {};
  float m_errorOld {};
  float m_errorSum {};
  unsigned long m_lastControlTime {};
};
