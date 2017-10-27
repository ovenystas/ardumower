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

/**
 * Setup sonar.
 *
 * @param triggerPin
 * @param echoPin
 * @param maxEchoTime
 * @param minEchoTime
 */
void sonar_setup(const uint8_t triggerPin, const uint8_t echoPin,
                 const uint16_t maxEchoTime, const uint16_t minEchoTime,
                 Sonar* sonar_p)
{
  sonar_p->triggerPin = triggerPin;
  sonar_p->echoPin = echoPin;
  sonar_p->maxEchoTime = maxEchoTime;
  sonar_p->minEchoTime = minEchoTime;

  digitalWrite(sonar_p->triggerPin, LOW);
  pinMode(sonar_p->triggerPin, OUTPUT);
  pinMode(sonar_p->echoPin, INPUT);

  sonar_p->triggerBitMask = digitalPinToBitMask(triggerPin); // Get the port register bit mask for the trigger pin.
  sonar_p->echoBitMask = digitalPinToBitMask(echoPin);       // Get the port register bit mask for the echo pin.
  sonar_p->triggerOutputRegister_p = portOutputRegister(digitalPinToPort(triggerPin)); // Get the output port register for the trigger pin.
  sonar_p->echoInputRegister_p = portInputRegister(digitalPinToPort(echoPin));         // Get the input port register for the echo pin.
}

inline bool sonar_pingTrigger(Sonar* sonar_p)
{
  delayMicroseconds(4);                        // Wait for pin to go low, testing shows it needs 4uS to work every time.
  *sonar_p->triggerOutputRegister_p |= sonar_p->triggerBitMask;  // Set trigger pin high, this tells the sensor to send out a ping.
  delayMicroseconds(10);                       // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
  *sonar_p->triggerOutputRegister_p &= (uint8_t)~sonar_p->triggerBitMask; // Set trigger pin back to low.

  sonar_p->maxTime =  micros() + MAX_SENSOR_DELAY;                                // Set a timeout for the ping to trigger.
  while ((*sonar_p->echoInputRegister_p & sonar_p->echoBitMask) && micros() <= sonar_p->maxTime)
  {
    // Wait for echo pin to clear.
  }
  while (!(*sonar_p->echoInputRegister_p & sonar_p->echoBitMask))                          // Wait for ping to start.
  {
    if (micros() > sonar_p->maxTime)
    {
      return false;                                // Something went wrong, abort.
    }
  }

  sonar_p->maxTime = micros() + sonar_p->maxEchoTime;    // Ping started, set the timeout.
  return true;                         // Ping started successfully.
}

inline uint16_t sonar_pingInternal(Sonar* sonar_p)
{
  if (!sonar_pingTrigger(sonar_p))  // Trigger a ping, if it returns false, return NO_ECHO to the calling function.
  {
    return NO_ECHO;
  }

  while (*sonar_p->echoInputRegister_p & sonar_p->echoBitMask)  // Wait for the ping echo.
  {
    if (micros() > sonar_p->maxTime)
    {
      return NO_ECHO;  // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
    }
  }
  return (uint16_t)(micros() - (sonar_p->maxTime - sonar_p->maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
}

/**
 * Send an ultrasound ping and measure time until echo arrives
 *
 * Sets distance_us to time until echo arrives in us.
 * To get distance in cm divide value with 58.8.
 */
void sonar_ping(Sonar* sonar_p)
{
  if (!sonar_p->use)
  {
    return;
  }

  sonar_p->distance_us = sonar_pingInternal(sonar_p);
}

bool sonars_isClose(Sonars* sonars_p)
{
  uint16_t closeLimit = (uint16_t)(sonars_p->triggerBelow * 2);
  for (uint8_t i = 0; i < SONAR_END; i++)
  {
    if (sonar_getDistance_us(&sonars_p->sonarArray_p[i]) < closeLimit)
    {
      return true;
    }
  }
  return false;
}
