/*
 * button.h
 *
 *  Created on: Mar 10, 2016
 *      Author: ove
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

typedef struct
{
  bool use;
  uint8_t pin;
  uint8_t counter;
  unsigned long lastRun;
} Button;

void button_setup(uint8_t pin, Button* button_p);
bool button_isTimeToRun(Button* button_p);

static inline
bool button_isPressed(Button* button_p)
{
  return (digitalRead(button_p->pin) == LOW);
}

static inline
uint8_t button_getCounter(Button* button_p)
{
  return button_p->counter;
}

static inline
void button_incCounter(Button* button_p)
{
  button_p->counter++;
}

static inline
void button_clearCounter(Button* button_p)
{
  button_p->counter = 0;
}

#endif /* BUTTON_H */
