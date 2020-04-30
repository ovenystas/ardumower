/*
 * button.h
 *
 *  Created on: Mar 10, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include "Setting.h"

struct ButtonSettings
{
  Setting<bool> use;                // Use the button or not
};

class Button
{
public:
  Button() = default;
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

  ButtonSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(ButtonSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
  }

private:
  uint8_t m_pin {};
  uint8_t m_counter {};
  unsigned long m_lastRun {};

  ButtonSettings m_settings
  {
    { "Use", true }
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
};
