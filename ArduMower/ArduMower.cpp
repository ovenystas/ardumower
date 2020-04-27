/*
 ArduMower (www.ardumower.de)
 Copyright (c) 2013-2015 by Alexander Grau
 Copyright (c) 2013-2015 by Sven Gennat
 Copyright (c) 2014 by Maxime Carpentieri
 Copyright (c) 2014-2015 by Stefan Manteuffel
 Copyright (c) 2015 by Uwe Zimprich

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


 changes: see https://code.google.com/p/ardumower/source/list

 Operation modes (press button for x beeps):
 1 beeps : normal mowing                           OK, tested
 2 beeps : normal mowing (bidir)                   broken
 3 beeps : drive by remote control (RC)            should work, not tested
 4 beeps : normal without perimeter                OK, tested
 5 beeps : find perimeter and track it             OK, tested
 6 beeps : track perimeter (debug-feature)         OK, tested
 7 beeps : normal mowing (lane-by-lane)            broken


 For additional circuits for button, buzzer etc. (DIY version), see www.ardumower.de

 */

// Do not remove the include below
#include "ArduMower.h"

#include <Wire.h>
#include <EEPROM.h>

#include "Robot.h"
#include "TaskConfig.h"


// requires: Arduino Mega

// Cooperative scheduler -------------------------------------------------------
static uint32_t tick = 0;          // System tick
static TaskType* task_p = nullptr; // Task pointer
static uint8_t numTasks = 0;       // Number of tasks
//------------------------------------------------------------------------------

Robot robot;

// odometer signal change interrupt
// mower motor speed sensor interrupt
// NOTE: when choosing a higher perimeter sample rate (38 kHz) and using odometer interrupts,
// the Arduino Mega cannot handle all ADC interrupts anymore - the result will be a 'noisy'
// perimeter filter output (mag value) which disappears when disabling odometer interrupts.
// SOLUTION: allow odometer interrupt handler nesting (see odometer interrupt function)
// http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
ISR(PCINT2_vect, ISR_NOBLOCK)
{
  robot.m_odometer.read();

  // TODO: Move this elsewhere
  robot.m_cutter.m_motor.setRpmState();
}

void setup()
{
  task_p = Task_getConfig();   // Get a pointer to the task configuration
  numTasks = Task_getNumTasks();

  robot.setup();
}


void loop()
{
  tick = millis();   // Get current system tick

  // Loop through all tasks. First, run all continuous tasks.
  // Then, if the number of ticks since the last time the task was run is
  //  greater than or equal to the task interval, execute the task.
  for (uint8_t idx = 0; idx < numTasks; idx++)
  {
    if (task_p[idx].interval == 0)
    {
      // Run continuous tasks.
      (*task_p[idx].func)();
    }
    else if ((tick - task_p[idx].lastTick) >= task_p[idx].interval)
    {
      (*task_p[idx].func)();        // Execute Task
      task_p[idx].lastTick = tick;  // Save last tick the task was ran.
    }
  }
}
