/*
 * encoder.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#include "Encoder.h"

void setupDefaults(Encoder* encoder_p)
{
  encoder_p->curState = LOW;
  encoder_p->curState2 = LOW;
  encoder_p->lastState = LOW;
  encoder_p->lastState2 = LOW;
  encoder_p->counter = 0;
  encoder_p->wheelRpmCurr = 0;
}

void encoder_setup2pin(const uint8_t pin, const uint8_t pin2,
    const bool swapDir,
    Encoder* encoder_p)
{
  encoder_p->twoWay = true;
  encoder_p->pin = pin;
  encoder_p->pin2 = pin2;
  encoder_p->swapDir = swapDir;

  pinMode(encoder_p->pin, INPUT_PULLUP);
  pinMode(encoder_p->pin2, INPUT_PULLUP);
}

void encoder_setup1pin(const uint8_t pin, const bool swapDir,
    Encoder* encoder_p)
{
  encoder_p->twoWay = false;
  encoder_p->pin = pin;
  encoder_p->swapDir = swapDir;

  pinMode(encoder_p->pin, INPUT_PULLUP);
}

void encoder_read(Encoder* encoder_p)
{
  encoder_p->curState = digitalRead(encoder_p->pin);
  if (encoder_p->twoWay)
  {
    encoder_p->curState2 = digitalRead(encoder_p->pin2);
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
void encoder_setState(Encoder* encoder_p)
{
  int8_t step = encoder_p->swapDir ? -1 : 1;

  if (encoder_p->curState != encoder_p->lastState)
  {
    if (encoder_p->curState)
    { // pin1 makes LOW->HIGH transition
      if (encoder_p->twoWay)
      {
        // pin2 = HIGH? => forward
        if (encoder_p->curState2)
        {
          encoder_p->counter += step;
        }
        else
        {
          encoder_p->counter -= step;
        }
      }
      else
      {
        if (encoder_p->wheelRpmCurr >= 0)
        {
          encoder_p->counter++;
        }
        else
        {
          encoder_p->counter--;
        }
      }
    }
    encoder_p->lastState = encoder_p->curState;
  }

  if (encoder_p->twoWay)
  {
    if (encoder_p->curState2 != encoder_p->lastState2)
    {
      encoder_p->lastState2 = encoder_p->curState2;
    }
  }
}
