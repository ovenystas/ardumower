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

typedef struct Pid_settingsT
{
  float Kp;         // proportional control
  float Ki;         // integral control
  float Kd;         // differential control
} Pid_settings_t;

typedef struct Pid
{
  Pid_settings_t settings;
  float setPoint;
  float yMin;
  float yMax;
  float maxOutput;
  float errorOld;
  float errorSum;
  unsigned long lastControlTime;
} Pid;

void pid_setup(float Kp, float Ki, float Kd,
               float yMin, float yMax, float maxOutput,
               Pid* pid_p);

float pid_compute(float processValue, Pid* pid_p);
