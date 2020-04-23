/*
 * sonar.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: ove
 */

#include "Sonar.h"

#define MAX_SENSOR_DELAY 18000
#define NO_ECHO 0

/**
 * Setup sonar.
 *
 * @param triggerPin
 * @param echoPin
 * @param maxEchoTime
 * @param minEchoTime
 */
void Sonar::setup(const uint8_t triggerPin, const uint8_t echoPin,
    const uint16_t maxEchoTime, const uint16_t minEchoTime)
{
  m_triggerPin = triggerPin;
  m_echoPin = echoPin;
  m_maxEchoTime = maxEchoTime;
  m_minEchoTime = minEchoTime;

  digitalWrite(m_triggerPin, LOW);
  pinMode(m_triggerPin, OUTPUT);
  pinMode(m_echoPin, INPUT);

  m_triggerBitMask = digitalPinToBitMask(triggerPin); // Get the port register bit mask for the trigger pin.
  m_echoBitMask = digitalPinToBitMask(echoPin);       // Get the port register bit mask for the echo pin.
  m_triggerOutputRegister_p = portOutputRegister(digitalPinToPort(triggerPin)); // Get the output port register for the trigger pin.
  m_echoInputRegister_p = portInputRegister(digitalPinToPort(echoPin));         // Get the input port register for the echo pin.
}

bool Sonar::pingTrigger()
{
  delayMicroseconds(4);                            // Wait for pin to go low, testing shows it needs 4uS to work every time.
  *m_triggerOutputRegister_p |= m_triggerBitMask;  // Set trigger pin high, this tells the sensor to send out a ping.
  delayMicroseconds(10);                           // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
  *m_triggerOutputRegister_p &= (uint8_t)~m_triggerBitMask; // Set trigger pin back to low.

  m_maxTime =  micros() + MAX_SENSOR_DELAY;                                // Set a timeout for the ping to trigger.
  while ((*m_echoInputRegister_p & m_echoBitMask) && micros() <= m_maxTime)
  {
    // Wait for echo pin to clear.
  }
  while (!(*m_echoInputRegister_p & m_echoBitMask))                          // Wait for ping to start.
  {
    if (micros() > m_maxTime)
    {
      return false;                                // Something went wrong, abort.
    }
  }

  m_maxTime = micros() + m_maxEchoTime; // Ping started, set the timeout.
  return true;                          // Ping started successfully.
}

uint16_t Sonar::pingInternal()
{
  if (!pingTrigger())  // Trigger a ping, if it returns false, return NO_ECHO to the calling function.
  {
    return NO_ECHO;
  }

  while (*m_echoInputRegister_p & m_echoBitMask)  // Wait for the ping echo.
  {
    if (micros() > m_maxTime)
    {
      return NO_ECHO;  // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
    }
  }
  return (uint16_t)(micros() - (m_maxTime - m_maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
}

/**
 * Send an ultrasound ping and measure time until echo arrives
 *
 * Sets distance_us to time until echo arrives in us.
 * To get distance in cm divide value with 58.8.
 */
void Sonar::ping()
{
  if (m_use)
  {
    m_distance_us = pingInternal();
  }
}

bool Sonars::isClose()
{
  uint16_t closeLimit = (uint16_t)(m_triggerBelow * 2);
  for (uint8_t i = 0; i < static_cast<uint8_t>(SonarE::END); i++)
  {
    if (m_sonarArray_p[i].getDistance_us() < closeLimit)
    {
      return true;
    }
  }
  return false;
}
