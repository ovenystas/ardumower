/*
 * Buzzer.h
 *
 *  Created on: Apr 3, 2020
 *      Author: ove
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

#ifndef TEST_VIRTUAL
#define TEST_VIRTUAL
#endif

enum class BeepType { LONG, SHORT };

struct BeepData
{
  uint16_t frequency;
  uint32_t duration_ms;
};

class Buzzer {
public:
  Buzzer(uint8_t pin, bool enabled = false) :
      m_pin(pin), m_enabled(enabled)
  {
    pinMode(m_pin, OUTPUT);
    digitalWrite(m_pin, LOW);
  }
  virtual ~Buzzer() {};

  bool isEnabled() const
  {
    return m_enabled;
  }

  void setEnabled(bool enabled)
  {
    m_enabled = enabled;
  }

  void beepShort(uint8_t numberOfBeeps = 1) const;
  void beepLong(uint8_t numberOfBeeps = 1) const;

  TEST_VIRTUAL void beep(uint16_t frequency, uint32_t duration_ms = 0) const;
  TEST_VIRTUAL void beep(const BeepData* data_p, uint8_t len = 1) const;

  TEST_VIRTUAL void beepStop() const;

private:
  uint8_t m_pin { };
  bool m_enabled { false };
};
