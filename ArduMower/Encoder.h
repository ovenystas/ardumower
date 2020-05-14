/*
 * encoder.h
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include <util/atomic.h>

#ifndef TEST_VIRTUAL
#define TEST_VIRTUAL
#endif

class Encoder
{
public:
  Encoder() = default;
  Encoder(uint8_t pin, bool swapDir)
  {
    setup(pin, swapDir);
  }
  TEST_VIRTUAL ~Encoder() {};

  TEST_VIRTUAL void setup(uint8_t pin, bool swapDir);

  TEST_VIRTUAL void read();

  TEST_VIRTUAL int16_t getCounter();
  TEST_VIRTUAL void clearCounter();

  TEST_VIRTUAL int16_t getWheelRpmCurr();
  TEST_VIRTUAL void setWheelRpmCurr(int16_t wheelRpmCurr);

public:
  bool m_swapDir {};  // invert encoder direction?

private:
  uint8_t m_pin {};          // encoder pin
  bool m_curState { LOW };   // current state
  bool m_lastState { LOW };  // last state
  int16_t m_counter {};      // wheel counter
  int16_t m_wheelRpmCurr {}; // wheel rpm
};
