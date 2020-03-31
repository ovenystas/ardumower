/*
 * encoder.h
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include <util/atomic.h>

class Encoder
{
public:
  Encoder() = default;
  Encoder(uint8_t pin, bool swapDir)
  {
    setup(pin, swapDir);
  }

  void setup(uint8_t pin, bool swapDir);

  void read();

  int16_t getCounter();
  void clearCounter();

  int16_t getWheelRpmCurr();
  void setWheelRpmCurr(int16_t wheelRpmCurr);

public:
  bool m_swapDir { false };  // invert encoder direction?

private:
  uint8_t m_pin {};           // encoder pin
  bool m_curState { false };  // current state
  bool m_lastState { false }; // last state
  int16_t m_counter {};       // wheel counter
  int16_t m_wheelRpmCurr {};  // wheel rpm
};
