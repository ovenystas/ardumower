/*
 * encoder.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#include "Encoder.h"

// Local function prototypes --------------------------------------------------


// Public functions -----------------------------------------------------------

void encoder_setup(uint8_t pin, bool swapDir, Encoder* encoder_p)
{
  encoder_p->curState = LOW;
  encoder_p->lastState = LOW;
  encoder_p->counter = 0;
  encoder_p->wheelRpmCurr = 0;
  encoder_p->pin = pin;
  encoder_p->swapDir = swapDir;

  pinMode(encoder_p->pin, INPUT_PULLUP);
}

void encoder_read(Encoder* encoder_p)
{
  encoder_p->curState = digitalRead(encoder_p->pin);

  int16_t step = encoder_p->swapDir ? -1 : 1;
  if (encoder_p->curState != encoder_p->lastState)
  {
    if (encoder_p->wheelRpmCurr >= 0)
    {
      encoder_p->counter = (int16_t)(encoder_p->counter + step);
    }
    else
    {
      encoder_p->counter = (int16_t)(encoder_p->counter - step);
    }
    encoder_p->lastState = encoder_p->curState;
  }
}

// Private functions ----------------------------------------------------------
