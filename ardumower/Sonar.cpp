/*
 * sonar.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: ove
 */

#include <Arduino.h>
#include "Sonar.h"

#define MAX_SENSOR_DELAY 18000
#define NO_ECHO 0

#define Console Serial

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
  this->triggerPin = triggerPin;
  this->echoPin = echoPin;
  this->maxEchoTime = maxEchoTime;
  this->minEchoTime = minEchoTime;

  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);

  triggerBitMask = digitalPinToBitMask(triggerPin); // Get the port register bit mask for the trigger pin.
  echoBitMask = digitalPinToBitMask(echoPin);       // Get the port register bit mask for the echo pin.
  triggerOutputRegister_p = portOutputRegister(digitalPinToPort(triggerPin)); // Get the output port register for the trigger pin.
  echoInputRegister_p = portInputRegister(digitalPinToPort(echoPin));         // Get the input port register for the echo pin.
}

/**
 * Send an ultrasound ping and measure time until echo arrives
 *
 * Sets distance_us to time until echo arrives in us.
 * To get distance in cm divide value with 58.8.
 */
void Sonar::ping(void)
{
  if (!use)
  {
    return;
  }

  distance_us = pingInternal();
}

inline bool Sonar::pingTrigger(void)
{
  delayMicroseconds(4);                        // Wait for pin to go low, testing shows it needs 4uS to work every time.
  *triggerOutputRegister_p |= triggerBitMask;  // Set trigger pin high, this tells the sensor to send out a ping.
  delayMicroseconds(10);                       // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
  *triggerOutputRegister_p &= ~triggerBitMask; // Set trigger pin back to low.

  maxTime =  micros() + MAX_SENSOR_DELAY;                                // Set a timeout for the ping to trigger.
  while ((*echoInputRegister_p & echoBitMask) && micros() <= maxTime)
  {
    // Wait for echo pin to clear.
  }
  while (!(*echoInputRegister_p & echoBitMask))                          // Wait for ping to start.
  {
    if (micros() > maxTime)
    {
      return false;                                // Something went wrong, abort.
    }
  }

  maxTime = micros() + maxEchoTime;    // Ping started, set the timeout.
  return true;                         // Ping started successfully.
}

inline uint32_t Sonar::pingInternal(void)
{
  if (!pingTrigger())  // Trigger a ping, if it returns false, return NO_ECHO to the calling function.
  {
    return NO_ECHO;
  }

  while (*echoInputRegister_p & echoBitMask)  // Wait for the ping echo.
  {
    if (micros() > maxTime)
    {
      return NO_ECHO;  // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
    }
  }
  return (micros() - (maxTime - maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
}

bool Sonars::isTimeToRun()
{
  unsigned long curMillis = millis();
  if (curMillis >= nextTime)
  {
    nextTime = curMillis + timeBetweenRun;
    return true;
  }
  return false;
}

bool Sonars::isTimeToCheck()
{
  unsigned long curMillis = millis();
  if (curMillis >= nextTimeCheck)
  {
    nextTimeCheck = curMillis + timeBetweenCheck;
    return true;
  }
  return false;
}
