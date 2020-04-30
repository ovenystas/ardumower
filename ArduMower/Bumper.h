/*
 * bumper.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include "Setting.h"

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

  void simHit();

  void clearHit()
  {
    m_hit = false;
  }

  bool isHit() const
  {
    return m_hit;
  }

  uint16_t getCounter() const
  {
    return m_counter;
  }

private:
  uint8_t m_pin { 0 };
  bool m_hit { false };
  uint16_t m_counter { 0 };
};

struct BumpersSettings
{
  Setting<bool> use; // Use this feature or not
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

  void check() const;
  bool isAnyHit() const;
  void clearHit() const;

  bool isUsed() const
  {
    return m_use;
  }

  BumpersSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(BumpersSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
  }

private:
  BumpersSettings m_settings
  {
    { "Use", false }
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;

  uint8_t m_len { 0 };
  Bumper* m_bumperArray_p { nullptr };
};
