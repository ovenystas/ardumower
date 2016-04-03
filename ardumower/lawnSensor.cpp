/*
 * lawnSensor.cpp
 *
 *  Created on: Apr 3, 2016
 *      Author: ove
 */

#include <Arduino.h>
#include "lawnSensor.h"

#define Console Serial

// measure lawn sensor capacity
uint16_t LawnSensor::measureLawnCapacity(const uint8_t pinSend,
                                         const uint8_t pinReceive)
{
  uint16_t t = 0;

  digitalWrite(pinSend, HIGH);

  // TODO: Improve this
  while (digitalRead(pinReceive) == LOW)
  {
    t++;
  }

  digitalWrite(pinSend, LOW);

  return t;
}

void LawnSensor::read()
{
  const float accel = 0.03;

  value[FRONT] = (1.0 - accel) * value[FRONT] +
      accel * (float)measureLawnCapacity(pinSend[FRONT], pinReceive[FRONT]);

  value[BACK]  = (1.0 - accel) * value[BACK] +
      accel * (float)measureLawnCapacity(pinSend[BACK],  pinReceive[BACK]);
}

void LawnSensor::check()
{
  float deltaFront = (value[FRONT] / valueOld[FRONT]) * 100.0;
  float deltaBack  = (value[BACK]  / valueOld[BACK])  * 100.0;
  if (deltaFront <= 95 || deltaBack <= 95)
  {
    Console.print(F("LAWN "));
    Console.print(deltaFront);
    Console.print(",");
    Console.println(deltaBack);
    counter++;
    detected = true;
  }
  valueOld[FRONT] = value[FRONT];
  valueOld[BACK] = value[BACK];
}
