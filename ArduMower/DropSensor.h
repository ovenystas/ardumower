#pragma once

#include <Arduino.h>
#include "Setting.h"

enum class DropSensor_Contact
{
  NO = 0,
  NC = 1
};

class DropSensor
{
public:
  DropSensor() = default;
  DropSensor(uint8_t pin) : m_pin(pin)
  {
    pinMode(m_pin, INPUT_PULLUP);
  }

  void setup(uint8_t pin);
  void check(DropSensor_Contact contactType);

  void simDetected()
  {
    m_detected = true;
    ++m_counter;
  }

  bool isDetected()
  {
    return m_detected;
  }

  void clearDetected()
  {
    m_detected = false;
  }

  uint16_t getCounter()
  {
    return m_counter;
  }

  void clearCounter()
  {
    m_counter = 0;
  }

private:
  uint8_t m_pin {};
  bool m_detected {};
  uint16_t m_counter {};
};

struct DropSensorsSettings
{
  Setting<bool> use;                // Use the drop sensors or not
};

class DropSensors
{
public:
  DropSensors() = default;
  DropSensors(DropSensor_Contact contactType, DropSensor* dropSensorArray_p,
      uint8_t len) :
        m_dropSensorArray_p(dropSensorArray_p),
        m_contactType(contactType),
        m_len(len) {};

  DropSensor_Contact getContactType()
  {
    return m_contactType;
  }

  void check();
  void clearDetected();

  bool isUsed() const
  {
    return m_use;
  }

  DropSensorsSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(DropSensorsSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
 }

public:
  DropSensor* m_dropSensorArray_p { nullptr };

private:
  DropSensor_Contact m_contactType {}; // contact 1=NC 0=NO against GND
  unsigned long m_lastRun {};
  uint8_t m_len {};

  DropSensorsSettings m_settings
  {
    { "Use", false }
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
};
