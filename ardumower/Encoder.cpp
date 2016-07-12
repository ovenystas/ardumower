/*
 * encoder.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#include "Encoder.h"

void Encoder::setup(const uint8_t pin, const uint8_t pin2, const bool swapDir)
{
  this->twoWay = true;
  this->pin = pin;
  this->pin2 = pin2;
  this->swapDir = swapDir;

  pinMode(pin, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
}

void Encoder::setup(const uint8_t pin, const bool swapDir)
{
  this->twoWay = false;
  this->pin = pin;
  this->swapDir = swapDir;

  pinMode(pin, INPUT_PULLUP);
}

void Encoder::read(void)
{
  curState = digitalRead(pin);
  if (twoWay)
  {
    curState2 = digitalRead(pin2);
  }
}

// ---- odometer (interrupt) --------------------------------------------------------
// Determines the rotation count and direction of the odometer encoders.
// Called in the odometer pins interrupt.
// Encoder signal/Ardumower pinout etc. at
// http://wiki.ardumower.de/index.php?title=Odometry
//
// Logic is:
//   If the pin1 change transition (odometerLeftState) is LOW -> HIGH...
//     If the pin2 current state is HIGH :  step count forward   (odometerLeft++)
//       Otherwise :  step count reverse   (odometerLeft--)
// odometerState:  1st odometer signal
// odometerState2: 2nd odometer signal (optional two-wire encoders)
void Encoder::setState(void)
{
  int8_t step = swapDir ? -1 : 1;

  if (curState != lastState)
  {
    if (curState)
    { // pin1 makes LOW->HIGH transition
      if (twoWay)
      {
        // pin2 = HIGH? => forward
        if (curState2)
        {
          counter += step;
        }
        else
        {
          counter -= step;
        }
      }
      else
      {
        if (wheelRpmCurr >= 0)
        {
          counter++;
        }
        else
        {
          counter--;
        }
      }
    }
    lastState = curState;
  }

  if (twoWay)
  {
    if (curState2 != lastState2)
    {
      lastState2 = curState2;
    }
  }
}
