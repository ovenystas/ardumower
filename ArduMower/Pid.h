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
#include "Setting.h"

/*
 digital PID controller
 */

struct PidSettings
{
  Setting<float> Kp; // Proportional control constant
  Setting<float> Ki; // Integral control constant
  Setting<float> Kd; // Differential control constant
};

class Pid
{
public:
  Pid() = default;
  Pid(float Kp, float Ki, float Kd, float yMin, float yMax, float maxOutput)
  {
    setup(Kp, Ki, Kd, yMin, yMax, maxOutput);
  }
  Pid(float Kp, float Ki, float Kd, float yMin, float yMax, float maxOutput,
      float regConstScale, float regConstMaxValue)
  {
    setup(Kp, Ki, Kd, yMin, yMax, maxOutput, regConstScale, regConstMaxValue);
  }

  void setup(float Kp, float Ki, float Kd, float yMin, float yMax,
      float maxOutput);

  void setup(float Kp, float Ki, float Kd, float yMin, float yMax,
      float maxOutput, float regConstScale, float regConstMaxValue);

  float compute(float processValue);

  float getSetPoint()
  {
    return m_setPoint;
  }

  void setSetPoint(float setPoint)
  {
    m_setPoint = setPoint;
  }

  PidSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(PidSettings* settings_p)
  {
    m_settings.Kp.value = settings_p->Kp.value;
    m_settings.Ki.value = settings_p->Ki.value;
    m_settings.Kd.value = settings_p->Kd.value;
  }

  float getErrorOld()
  {
    return m_errorOld;
  }

private:
  float m_setPoint {};
  float m_yMin {};
  float m_yMax {};
  float m_maxOutput {};
  float m_errorOld {};
  float m_errorSum {};
  unsigned long m_lastControlTime {};

  PidSettings m_settings
  {
    { "Kp", "", 1.0f, 0.0f, 100.0f, 0.1f },
    { "Ki", "", 1.0f, 0.0f, 100.0f, 0.1f },
    { "Kd", "", 1.0f, 0.0f, 100.0f, 0.1f }
  };

  // Shorter convenient variables for settings variables
  float& m_kp = m_settings.Kp.value;
  float& m_ki = m_settings.Ki.value;
  float& m_kd = m_settings.Kd.value;
};
