#pragma once

#include <Arduino.h>

enum class DropSensor_Contact
{
  NO = 0,
  NC = 1
};

class DropSensor
{
public:
  DropSensor() = default;
  DropSensor(uint8_t pin)
  {
    setup(pin);
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

class DropSensors
{
public:
  DropSensors() = default;
  DropSensors(const uint8_t* pins_p, DropSensor_Contact contactType,
      DropSensor* dropSensorArray_p, uint8_t len)
  {
    setup(pins_p, contactType, dropSensorArray_p, len);
  }

  void setup(const uint8_t* pins, DropSensor_Contact contactType,
      DropSensor* dropSensorArray_p, uint8_t len);

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

public:
  bool m_use {};
  DropSensor* m_dropSensorArray_p { nullptr };

private:
  DropSensor_Contact m_contactType {}; // contact 1=NC 0=NO against GND
  unsigned long m_lastRun {};
  uint8_t m_len {};
};
