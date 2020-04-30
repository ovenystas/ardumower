/**
 * HC-SR04 ultrasonic sensor driver
 *
 * Features:
 *   Power Supply :+5V DC
 *   Quiescent Current : <2mA
 *   Working Current: 15mA
 *   Effectual Angle: <15°
 *   Ranging Distance : 2cm – 400 cm/1″ – 13ft
 *   Resolution : 0.3 cm
 *   Measuring Angle: 30 degree
 *   Trigger Input Pulse width: 10uS
 *   Dimension: 45mm x 20mm x 15mm
 */
#pragma once

#include <Arduino.h>
#include "Setting.h"

#define SONAR_DEFAULT_MAX_ECHO_TIME 3000  // 3000 us / 58.8 = 51 cm
#define SONAR_DEFAULT_MIN_ECHO_TIME 150   // 150 us / 58,8 = 2.5 cm

// Conversion from uS to distance (round result to nearest cm or inch).
#define US_ROUNDTRIP_INCH 146u // Microseconds (uS) it takes sound to travel round-trip 1 inch (2 inches total), uses integer to save compiled code space.
#define US_ROUNDTRIP_CM 57u    // Microseconds (uS) it takes sound to travel round-trip 1 cm (2 cm total), uses integer to save compiled code space.
#define PING_CONVERT(us, conversionFactor) (uint16_t)(max((us + conversionFactor / 2u) / conversionFactor, (us ? 1u : 0u)))

enum class SonarE
{
  LEFT,
  CENTER,
  RIGHT,
  END
};

struct SonarSettings
{
  Setting<bool> use;                // Use the sonar sensor or not
};

class Sonar
{
public:
  Sonar() = default;
  Sonar(const uint8_t triggerPin, const uint8_t echoPin,
      const uint16_t maxEchoTime = SONAR_DEFAULT_MAX_ECHO_TIME,
      const uint16_t minEchoTime = SONAR_DEFAULT_MIN_ECHO_TIME)
  {
    setup(triggerPin, echoPin, maxEchoTime, minEchoTime);
  }

  void setup(const uint8_t triggerPin, const uint8_t echoPin,
      const uint16_t maxEchoTime = SONAR_DEFAULT_MAX_ECHO_TIME,
      const uint16_t minEchoTime = SONAR_DEFAULT_MIN_ECHO_TIME);

  bool isUsed()
  {
    return m_use;
  }

  void ping();


  uint16_t getDistance_us()
  {
    return m_distance_us;
  }


  uint16_t getDistance_cm()
  {
    return PING_CONVERT(m_distance_us, US_ROUNDTRIP_CM);
  }


  uint16_t getDistance_inch()
  {
    return PING_CONVERT(m_distance_us, US_ROUNDTRIP_INCH);
  }

  SonarSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(SonarSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
 }

private:
  bool pingTrigger();
  uint16_t pingInternal();

private:
  uint8_t m_triggerPin {};
  uint8_t m_echoPin {};
  uint16_t m_maxEchoTime {};
  uint16_t m_minEchoTime {};
  uint16_t m_distance_us {};   // As time in us
  uint8_t m_triggerBitMask {};
  uint8_t m_echoBitMask {};
  volatile uint8_t* m_triggerOutputRegister_p {};
  volatile uint8_t* m_echoInputRegister_p {};
  uint32_t m_maxTime {};

  SonarSettings m_settings
  {
    { "Use", false }
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
};

struct SonarsSettings
{
  Setting<bool> use;              // Use the sonar sensors or not
  Setting<uint16_t> triggerBelow; // Trigger distance in us
};

class Sonars
{
public:
  Sonars() = default;
  Sonars(Sonar* sonarArray_p, const uint8_t len) :
    m_sonarArray_p(sonarArray_p),
    m_len(len)
  {};

  bool isUsed()
  {
    return m_use;
  }

  void ping()
  {
    for (uint8_t i = 0; i < static_cast<uint8_t>(SonarE::END); i++)
    {
      m_sonarArray_p[i].ping();
    }
  }


  uint16_t getDistanceCounter()
  {
    return m_distanceCounter;
  }


  void incDistanceCounter()
  {
    ++m_distanceCounter;
  }

  bool isClose();

  SonarsSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(SonarsSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
    m_settings.triggerBelow.value = settings_p->triggerBelow.value;
  }

  uint32_t getObstacleTimeout() const
  {
    return m_obstacleTimeout;
  }

  void setObstacleTimeout(uint32_t obstacleTimeout)
  {
    m_obstacleTimeout = obstacleTimeout;
  }

  uint8_t getTempDistanceCounter() const
  {
    return m_tempDistanceCounter;
  }

  void incTempDistanceCounter()
  {
    ++m_tempDistanceCounter;
  }

  void clearTempDistanceCounter()
  {
    m_tempDistanceCounter = 0;
  }

public:
  // TODO: Make this private
  Sonar* m_sonarArray_p {};

private:
  uint8_t m_tempDistanceCounter {};
  uint32_t m_obstacleTimeout {};
  uint8_t m_len {};
  uint16_t m_distanceCounter {};

  SonarsSettings m_settings
  {
    { "Use", false },
    { "Trigger below", "us", 1050, 1, 3000 }
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
  uint16_t& m_triggerBelow = m_settings.triggerBelow.value;

};
