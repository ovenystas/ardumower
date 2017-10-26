/*
 * lawnSensor.cpp
 *
 *  Created on: Apr 3, 2016
 *      Author: ove
 */

#include "LawnSensor.h"

void lawnSensor_setup(const uint8_t sendPin, const uint8_t receivePin,
                      LawnSensor* lawnSensor_p)
{
  lawnSensor_p->sendPin = sendPin;
  lawnSensor_p->receivePin = receivePin;

  pinMode(lawnSensor_p->sendPin, OUTPUT);
  pinMode(lawnSensor_p->receivePin, INPUT);

  lawnSensor_p->value = 0;
  lawnSensor_p->valueOld = 0;
}


uint16_t lawnSensor_measureLawnCapacity(LawnSensor* lawnSensor_p)
{
  digitalWrite(lawnSensor_p->sendPin, HIGH);

  // TODO: Improve this
  uint16_t t = 0;
  while (digitalRead(lawnSensor_p->receivePin) == LOW)
  {
    t++;
  }

  digitalWrite(lawnSensor_p->sendPin, LOW);

  return t;
}

void lawnSensor_read(LawnSensor* lawnSensor_p)
{
  const float accel = 0.03f;

  lawnSensor_p->value = (1.0f - accel) * lawnSensor_p->value +
      accel * (float)lawnSensor_measureLawnCapacity(lawnSensor_p);
}

void lawnSensors_check(LawnSensors* lawnSensors_p)
{
  LawnSensor* sensorF_p = &lawnSensors_p->lawnSensorArray_p[0];
  LawnSensor* sensorB_p = &lawnSensors_p->lawnSensorArray_p[1];

  float deltaF = (sensorF_p->value / sensorF_p->valueOld) * 100.0f;
  float deltaB = (sensorB_p->value / sensorB_p->valueOld) * 100.0f;

  if (deltaF <= 95 || deltaB <= 95)
  {
    lawnSensors_p->counter++;
    lawnSensors_p->detected = true;
  }

  sensorF_p->valueOld = sensorF_p->value;
  sensorB_p->valueOld = sensorB_p->value;
}
