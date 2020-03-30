/*
 * button.h
 *
 *  Created on: Mar 10, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>

class Button
{
public:
  Button(uint8_t pin) : m_pin{pin}
  {
    pinMode(m_pin, INPUT_PULLUP);
  };

  bool isTimeToRun();

  bool isUsed() const
  {
    return m_use;
  }

  bool isPressed() const
  {
    return (digitalRead(m_pin) == LOW);
  }

  uint8_t getCounter() const
  {
    return m_counter;
  }

  void incCounter()
  {
    m_counter++;
  }

  void clearCounter()
  {
    m_counter = 0;
  }

  bool m_use = true;

private:
  uint8_t m_pin = 0;
  uint8_t m_counter = 0;
  unsigned long m_lastRun = 0;
};
