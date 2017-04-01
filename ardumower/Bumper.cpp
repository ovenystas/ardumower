/*
 * Bumper.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "Bumper.h"
#include "Config.h"

static bool hit[NUM_BUMPERS] = {false};
static uint16_t counter[NUM_BUMPERS] = {0};
static const uint8_t pin[NUM_BUMPERS] = {PIN_BUMBER_LEFT, PIN_BUMBER_RIGHT};
bool bumpers_use = false;

void bumper_simHit(uint8_t side)
{
  hit[side] = true;
  counter[side]++;
}

bool bumper_isHit(uint8_t side)
{
  return hit[side];
}

uint16_t bumper_getCounter(uint8_t side)
{
  return counter[side];
}

void bumpers_setup(void)
{
  pinMode(pin[BUMPER_LEFT], INPUT_PULLUP);
  pinMode(pin[BUMPER_RIGHT], INPUT_PULLUP);
}

void bumpers_check(void)
{
  if (digitalRead(pin[BUMPER_LEFT]) == LOW)
  {
    hit[BUMPER_LEFT] = true;
    counter[BUMPER_LEFT]++;
  }
  if (digitalRead(pin[BUMPER_RIGHT]) == LOW)
  {
    hit[BUMPER_RIGHT] = true;
    counter[BUMPER_RIGHT]++;
  }
}

void bumpers_clearHit(void)
{
  hit[BUMPER_LEFT] = false;
  hit[BUMPER_RIGHT] = false;
}

bool bumpers_isAnyHit(void)
{
  return hit[BUMPER_LEFT] || hit[BUMPER_RIGHT];
}
