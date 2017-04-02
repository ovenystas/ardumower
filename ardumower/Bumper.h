/*
 * bumper.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ove
 */

#ifndef BUMPER_H
#define BUMPER_H

#include <Arduino.h>

typedef struct
{
  uint8_t pin;
  bool hit;
  uint16_t counter;
} Bumper;

typedef struct
{
  bool use;
  uint8_t len;
  Bumper* bumperArray_p;
} Bumpers;

void bumper_setup(const uint8_t pin, Bumper* bumper_p);
void bumper_check(Bumper* bumper_p);
bool bumpers_isAnyHit(const Bumpers* bumpers_p);

static inline
void bumper_simHit(Bumper* bumper_p)
{
  bumper_p->hit = true;
  bumper_p->counter++;
}

static inline
void bumper_clearHit(Bumper* bumper_p)
{
  bumper_p->hit = false;
}

static inline
bool bumper_isHit(const Bumper* bumper_p)
{
  return bumper_p->hit;
}

static inline
uint16_t bumper_getCounter(const Bumper* bumper_p)
{
  return bumper_p->counter;
}

static inline
void bumpers_setup(const uint8_t* pins, Bumper* bumperArray_p,
                   Bumpers* bumpers_p, const uint8_t len)
{
  bumpers_p->len = len;
  bumpers_p->bumperArray_p = bumperArray_p;
  for (uint8_t i = 0; i < len; i++)
  {
    bumper_setup(pins[i], &bumperArray_p[i]);
  }
}

static inline
void bumpers_check(Bumpers* bumpers_p)
{
  for (uint8_t i = 0; i < bumpers_p->len; i++)
  {
    bumper_check(&bumpers_p->bumperArray_p[i]);
  }
}

static inline
void bumpers_clearHit(Bumpers* bumpers_p)
{
  for (uint8_t i = 0; i < bumpers_p->len; i++)
  {
    bumper_clearHit(&bumpers_p->bumperArray_p[i]);
  }
}

#endif // BUMPER_H
