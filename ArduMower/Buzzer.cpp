/*
 * Buzzer.cpp
 *
 *  Created on: Apr 3, 2020
 *      Author: ove
 */

#include "Buzzer.h"

class BuzzerConst
{
public:
  static const uint16_t frequency_hz = 4200;
  static const uint32_t durationShort_ms = 50;
  static const uint32_t durationLong_ms = 500;
  static const uint32_t delayShort_ms = 250;
  static const uint32_t delayLong_ms = 500;
};

void Buzzer::beepShort(uint8_t numberOfBeeps)
{
  if (!isEnabled()) return;

  for (uint8_t i = 0; i < numberOfBeeps; i++)
  {
    tone(m_pin, BuzzerConst::frequency_hz, BuzzerConst::durationShort_ms);
    delay(BuzzerConst::delayShort_ms);
  }
}

void Buzzer::beepLong(uint8_t numberOfBeeps)
{
  if (!isEnabled()) return;

  for (uint8_t i = 0; i < numberOfBeeps; i++)
  {
    tone(m_pin, BuzzerConst::frequency_hz, BuzzerConst::durationLong_ms);
    delay(BuzzerConst::delayLong_ms);
  }
}

void Buzzer::beep(BeepData* data_p, uint8_t len)
{
  if (!isEnabled()) return;

  for (uint8_t i = 0; i < len; i++)
  {
    tone(m_pin, data_p[i].frequency, data_p[i].duration_ms);
  }
}
