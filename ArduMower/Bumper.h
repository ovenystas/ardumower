/*
 * bumper.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>

class Bumper
{
public:
  Bumper() = default;
  Bumper(uint8_t pin)
  {
    setup(pin);
  }

  void setup(uint8_t pin)
  {
    m_pin = pin;
    m_hit = false;
    m_counter = 0;
    pinMode(m_pin, INPUT_PULLUP);
  }

  void check();

  void simHit()
  {
    m_hit = true;
    m_counter++;
  }

  void clearHit() { m_hit = false; }
  bool isHit() { return m_hit; }
  uint16_t getCounter() { return m_counter; }

private:
  uint8_t m_pin { 0 };
  bool m_hit { false };
  uint16_t m_counter { 0 };
};

class Bumpers
{
public:
  Bumpers() = default;
  Bumpers(const uint8_t* pins, Bumper* bumperArray_p, uint8_t len)
  {
    setup(pins, bumperArray_p, len);
  }

  void setup(const uint8_t* pins, Bumper* bumperArray_p, uint8_t len);

  void check();
  bool isAnyHit();
  void clearHit();

  bool isUsed() { return m_use; }

  bool m_use { false };

private:
  uint8_t m_len { 0 };
  Bumper* m_bumperArray_p { nullptr };
};
