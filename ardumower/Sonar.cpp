/*
 * sonar.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: ove
 */

#include <Arduino.h>

#include "Sonar.h"

// ultrasonic sensor max echo time
// (WARNING: do not set too high, it consumes CPU time!)
#define DEFAULT_MAX_ECHO_TIME 3000  // 3000 us / 58.8 = 51 cm
#define DEFAULT_MIN_ECHO_TIME 150   // 150 us / 58,8 = 2.5 cm
#define NO_ECHO 0

/**
 * Setup sonar using default values for max and min echo times.
 *
 * @param triggerPin
 * @param echoPin
 */
void Sonar::setup(const uint8_t triggerPin, const uint8_t echoPin)
{
  setup(triggerPin, echoPin, DEFAULT_MAX_ECHO_TIME, DEFAULT_MIN_ECHO_TIME);
}

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

  pinMode(this->triggerPin, OUTPUT);
  pinMode(this->echoPin, INPUT);
}

/**
 * Send an ultrasound ping and measure time until echo arrives
 *
 * @return Time until echo arrives in us.
 *         To get distance in cm divide value with 58.8.
 */
void Sonar::ping(void)
{
  if (!use)
  {
    return;
  }

  // TODO: Change from digitalWrite to raw port write to make it more accurate.
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(4);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Duration is in us
  uint32_t uS = pulseIn(echoPin, HIGH, maxEchoTime + 1000);

  if (uS > maxEchoTime || uS < minEchoTime)
  {
    uS = NO_ECHO;
  }

  distance = uS;
}
