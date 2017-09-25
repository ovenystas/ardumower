/*
 * encoder.h
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

typedef struct
{
  uint8_t pin;           // 1st encoder pin
  uint8_t pin2;          // 2nd encoder pin
  bool twoWay;           // using 2-pin encoder
  bool swapDir;          // invert encoder direction?
  bool curState;         // current state
  bool curState2;        // current state on 2nd pin
  bool lastState;        // last state
  bool lastState2;       // last state on 2nd pin
  int16_t counter;       // wheel counter
  int16_t wheelRpmCurr;  // wheel rpm
} Encoder;

// Setup for one-way encoder (1-pin)
void encoder_setup1pin(const uint8_t pin, const bool swapDir,
    Encoder* encoder_p);

// Setup for two-way encoder (2-pins)
void encoder_setup2pin(const uint8_t pin, const uint8_t pin2,
    const bool swapDir,
    Encoder* encoder_p);

void encoder_read(Encoder* encoder_p);
void encoder_setState(Encoder* encoder_p);

static inline int16_t encoder_getCounter(Encoder* encoder_p)
{
  return encoder_p->counter;
}

static inline
void encoder_clearCounter(Encoder* encoder_p)
{
  encoder_p->counter = 0;
}

static inline int16_t encoder_getWheelRpmCurr(Encoder* encoder_p)
{
  return encoder_p->wheelRpmCurr;
}

static inline
void encoder_setWheelRpmCurr(int16_t wheelRpmCurr, Encoder* encoder_p)
{
  encoder_p->wheelRpmCurr = wheelRpmCurr;
}

#endif /* ENCODER_H */
