/*
 * encoder.h
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include <util/atomic.h>

typedef struct
{
  uint8_t pin;           // encoder pin
  bool swapDir;          // invert encoder direction?
  bool curState;         // current state
  bool lastState;        // last state
  int16_t counter;       // wheel counter
  int16_t wheelRpmCurr;  // wheel rpm
} Encoder;

void encoder_setup(const uint8_t pin, const bool swapDir, Encoder* encoder_p);

void encoder_read(Encoder* encoder_p);

static inline
int16_t encoder_getCounter(Encoder* encoder_p)
{
  int16_t counter;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    counter = encoder_p->counter;
  }
  return counter;
}

static inline
void encoder_clearCounter(Encoder* encoder_p)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    encoder_p->counter = 0;
  }
}

static inline
int16_t encoder_getWheelRpmCurr(Encoder* encoder_p)
{
  int16_t rpm;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    rpm = encoder_p->wheelRpmCurr;
  }
  return rpm;
}

static inline
void encoder_setWheelRpmCurr(int16_t wheelRpmCurr, Encoder* encoder_p)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    encoder_p->wheelRpmCurr = wheelRpmCurr;
  }
}
