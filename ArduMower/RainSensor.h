/*
 * rainSensor.h
 *
 *  Created on: Apr 7, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include "Setting.h"

struct RainSensorSettings
{
  Setting<bool> use;                // Use the rain sensor or not
};

class RainSensor
{
public:
  RainSensor() = default;
  RainSensor(const uint8_t pin)
  {
    setup(pin);
  }

  void setup(const uint8_t pin);

  bool isUsed()
  {
    return m_use;
  }

  void check();

  bool isRaining()
  {
    return m_raining;
  }

  uint16_t getCounter()
  {
    return m_counter;
  }

  RainSensorSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(RainSensorSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
 }

private:
  uint8_t m_pin {};
  bool m_raining {};
  uint16_t m_counter {};

  RainSensorSettings m_settings
  {
    { "Use", "", false, false, true }
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
};

