/*
 * lawnSensor.h
 *
 *  Created on: Apr 3, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>
#include "Setting.h"

class LawnSensor
{
public:
  LawnSensor() = default;
  LawnSensor(const uint8_t sendPin, const uint8_t receivePin)
  {
    setup(sendPin, receivePin);
  }

  void setup(uint8_t sendPin, uint8_t receivePin);

  void read();

  float getValue()
  {
    return m_value;
  }

  uint16_t measureLawnCapacity();

public:
  // TODO: Make these private
  float m_value {};     // lawn sensor capacity (time)
  float m_valueOld {};  // lawn sensor capacity (time)

private:
  uint8_t m_sendPin {};
  uint8_t m_receivePin {};
};

struct LawnSensorsSettings
{
  Setting<bool> use;                // Use the lawn sensors or not
};

class LawnSensors
{
public:
  LawnSensors() = default;
  LawnSensors(LawnSensor* lawnSensorArray_p, const uint8_t len) :
    m_lawnSensorArray_p(lawnSensorArray_p),
    m_len(len)
  {};

  bool isUsed()
  {
    return m_use;
  }

  void read()
  {
    for (uint8_t i = 0; i < m_len; i++)
    {
      m_lawnSensorArray_p[i].read();
    }
  }

  bool isDetected()
  {
    return m_detected;
  }

  void clearDetected()
   {
    m_detected = false;
   }

  void simDetected()
  {
    m_detected = true;
    m_counter++;
  }

  uint16_t getCounter()
  {
    return m_counter;
  }

  void check();

  LawnSensorsSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(LawnSensorsSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
 }

private:
  LawnSensor* m_lawnSensorArray_p  {};
  uint8_t m_len {};
  bool m_detected { false };
  uint16_t m_counter {};

  LawnSensorsSettings m_settings
  {
    { "Use", "", false, false, true }
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
};
