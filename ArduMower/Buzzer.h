/*
 * Buzzer.h
 *
 *  Created on: Apr 3, 2020
 *      Author: ove
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

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

  bool isEnabled()
  {
    return m_enabled;
  }

  void setEnabled(bool enabled)
  {
    m_enabled = enabled;
  }

  void beepShort(uint8_t numberOfBeeps);
  void beepLong(uint8_t numberOfBeeps);

  void beep(uint16_t frequency, uint32_t duration_ms = 0)
  {
    tone(m_pin, frequency, duration_ms);
  }

  void beep(BeepData* data_p, uint8_t len = 1);

  void beepStop()
  {
    noTone(m_pin);
  }

private:
  uint8_t m_pin { };
  bool m_enabled { false };
};
