/*
 * bumper.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ove
 */

#ifndef BUMPER_H
#define BUMPER_H

#include <Arduino.h>

enum bumpersE
{
  BUMPER_LEFT,
  BUMPER_RIGHT,
  NUM_BUMPERS
};

extern bool bumpers_use;

void bumper_simHit(uint8_t side);
bool bumper_isHit(uint8_t side);
uint16_t bumper_getCounter(uint8_t side);

void bumpers_setup(void);
void bumpers_check(void);
bool bumpers_isAnyHit(void);
void bumpers_clearHit(void);

#endif // BUMPER_H
