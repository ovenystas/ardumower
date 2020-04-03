/*
 * Led.h
 *
 *  Created on: Apr 3, 2020
 *      Author: ove
 */
#pragma once

#include <Arduino.h>

class Led
{
public:
  Led(uint8_t pin, uint8_t initState = LOW) : m_pin(pin)
  {
    pinMode(m_pin, OUTPUT);
    digitalWrite(m_pin, initState);
  }

  void set(uint8_t val) { digitalWrite(m_pin, val); }
  void setLow() { digitalWrite(m_pin, LOW); }
  void setHigh() { digitalWrite(m_pin, HIGH); }

  bool get() { return static_cast<bool>(digitalRead(m_pin)); }

  bool toggle()
  {
    bool newVal = !get();
    set(newVal);
    return newVal;
  }

private:
  uint8_t m_pin {};
};
